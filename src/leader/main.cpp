// SO101 リーダー側: 関節角を読み取り BLE Notify でフォロワーへ送信する
// BLE ペリフェラル (GATT サーバ) として動作
#include <Arduino.h>
#include <NimBLEDevice.h>

#include "../common/peer_store.h"
#include "../common/so101_bus.h"
#include "../common/so101_motion.h"
#include "../common/ui.h"
#include "so101_link.h"

// ---- 画面モード -------------------------------------------------------------
enum class Mode { Run, Menu, CalibConfirm, Pairing, Message };
static Mode g_mode = Mode::Run;
static int g_menuSel = 0;
static uint32_t g_msgUntil = 0;
static const char* const kMenuItems[] = {"Calibrate", "Pairing", "Back"};
static constexpr int kMenuCount = sizeof(kMenuItems) / sizeof(kMenuItems[0]);

// ---- BLE 状態 ---------------------------------------------------------------
static NimBLEServer* g_server = nullptr;
static NimBLECharacteristic* g_jointChr = nullptr;
static bool g_hasPeer = false;
static NimBLEAddress g_peer;

static constexpr uint16_t kNoConn = 0xFFFF;
static volatile uint16_t g_connHandle = kNoConn;  // 許可済みピアの接続
static volatile bool g_pairing = false;
static volatile bool g_needAdvUpdate = false;
// ペアリング要求 (コールバック → loop)
static volatile bool g_pairRequested = false;
static NimBLEAddress g_pairAddr;
// ペアリング中に接続してきたが要求を書かない相手を切るため
static volatile uint16_t g_pendingHandle = kNoConn;
static volatile uint32_t g_pendingSince = 0;
static constexpr uint32_t kPendingTimeoutMs = 3000;

// ---- 関節 -------------------------------------------------------------------
static Unwrapper g_unwrap[SO101_NUM_JOINTS];
static uint8_t g_servoMask = 0;
static uint16_t g_lastPos[SO101_NUM_JOINTS];
static bool g_posValid = false;
static uint8_t g_warnMask = 0;
static uint16_t g_seq = 0;
static uint32_t g_sentPerSec = 0, g_errPerSec = 0;

// ---------------------------------------------------------------------------
// BLE
// ---------------------------------------------------------------------------
static void updateConnParams(uint16_t handle) {
  g_server->updateConnParams(handle, SO101_CONN_INTERVAL_MIN,
                             SO101_CONN_INTERVAL_MAX, SO101_CONN_LATENCY,
                             SO101_SUPERVISION_TIMEOUT);
}

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, ble_gap_conn_desc* desc) override {
    const NimBLEAddress addr(desc->peer_id_addr);
    if (g_pairing) {
      // ペアリング要求の書き込みを待つ
      g_pendingHandle = desc->conn_handle;
      g_pendingSince = millis();
      return;
    }
    if (g_hasPeer && addr == g_peer && g_connHandle == kNoConn) {
      g_connHandle = desc->conn_handle;
      updateConnParams(desc->conn_handle);
      return;
    }
    server->disconnect(desc->conn_handle);  // ペア以外は拒否
  }
  void onDisconnect(NimBLEServer* server, ble_gap_conn_desc* desc) override {
    if (desc->conn_handle == g_connHandle) g_connHandle = kNoConn;
    if (desc->conn_handle == g_pendingHandle) g_pendingHandle = kNoConn;
    g_needAdvUpdate = true;
  }
};

class CtrlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* chr, ble_gap_conn_desc* desc) override {
    const std::string v = chr->getValue();
    if (!g_pairing || v.size() != 1 || (uint8_t)v[0] != SO101_CTRL_PAIR_REQ) {
      return;
    }
    // 最初に要求してきた相手とペアリング
    g_pairing = false;
    g_pairAddr = NimBLEAddress(desc->peer_id_addr);
    g_connHandle = desc->conn_handle;
    g_pendingHandle = kNoConn;
    updateConnParams(desc->conn_handle);
    g_pairRequested = true;
  }
};

// 現在のモードに合わせてアドバタイズを (再) 開始
static void updateAdvertising() {
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->stop();
  const bool want = g_connHandle == kNoConn && g_pendingHandle == kNoConn &&
                    (g_pairing || g_hasPeer);
  if (!want) return;  // 未ペア or 接続中はアドバタイズしない
  const uint8_t mfg[SO101_ADV_LEN] = {
      0xFF, 0xFF, SO101_ADV_MAGIC0, SO101_ADV_MAGIC1,
      g_pairing ? SO101_ADV_MODE_PAIRING : SO101_ADV_MODE_NORMAL};
  adv->setManufacturerData(std::string((const char*)mfg, sizeof(mfg)));
  adv->start();
}

static void disconnectAll() {
  for (uint16_t h : g_server->getPeerDevices()) g_server->disconnect(h);
}

static void setupBle() {
  NimBLEDevice::init(SO101_LEADER_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  g_server = NimBLEDevice::createServer();
  g_server->setCallbacks(new ServerCallbacks());
  g_server->advertiseOnDisconnect(false);  // 自前で制御する

  NimBLEService* svc = g_server->createService(SO101_SERVICE_UUID);
  g_jointChr = svc->createCharacteristic(
      SO101_JOINT_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic* ctrl =
      svc->createCharacteristic(SO101_CTRL_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
  ctrl->setCallbacks(new CtrlCallbacks());
  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SO101_SERVICE_UUID);
  adv->setScanResponse(true);

  g_hasPeer = peerstore::load(g_peer);
  updateAdvertising();
}

static void startPairing() {
  peerstore::clear();  // ペアリングのたびにクリア
  g_hasPeer = false;
  g_pairing = true;
  disconnectAll();
  updateAdvertising();
}

static void cancelPairing() {
  g_pairing = false;
  if (g_pendingHandle != kNoConn) g_server->disconnect(g_pendingHandle);
  updateAdvertising();
}

// ---------------------------------------------------------------------------
// サーボ
// ---------------------------------------------------------------------------
// 一部の関節が読めなかった場合は前回値を使う。全関節が一度でも読めるまでは送らない。
static bool readLeaderJoints(uint16_t pos[SO101_NUM_JOINTS]) {
  static uint8_t everRead = 0;
  uint16_t raw[SO101_NUM_JOINTS];
  const uint8_t mask = so101bus::readPositions(raw);
  if (mask != SO101_ALL_JOINTS) g_errPerSec++;
  g_warnMask = 0;
  for (int i = 0; i < SO101_NUM_JOINTS; i++) {
    if (mask & (1 << i)) g_lastPos[i] = g_unwrap[i].update(raw[i]);
    if (g_unwrap[i].saturated()) g_warnMask |= 1 << i;
  }
  everRead |= mask;
  g_posValid = everRead == SO101_ALL_JOINTS;
  if (!g_posValid) return false;
  memcpy(pos, g_lastPos, sizeof(g_lastPos));
  return true;
}

// ---------------------------------------------------------------------------
// 画面
// ---------------------------------------------------------------------------
static void showMessage(const char* title, uint16_t color, const char* l1,
                        const char* l2 = nullptr, const char* l3 = nullptr,
                        const char* l4 = nullptr) {
  ui::drawMessage(title, color, l1, l2, l3, l4, "click: close");
  g_mode = Mode::Message;
  g_msgUntil = millis() + 5000;
}

static void drawRun() {
  ui::RunView v{};
  v.role = "LEADER";
  v.roleColor = TFT_ORANGE;
  if (!g_hasPeer) {
    v.link = "NOT PAIRED";
    v.linkColor = TFT_DARKGREY;
  } else if (g_connHandle != kNoConn) {
    v.link = "CONNECTED";
    v.linkColor = TFT_GREEN;
  } else {
    v.link = "WAITING FOLLOWER";
    v.linkColor = TFT_YELLOW;
  }
  snprintf(v.peer, sizeof(v.peer), "%s",
           g_hasPeer ? g_peer.toString().c_str() : "-");
  snprintf(v.stat, sizeof(v.stat), "tx %3lu/s err %lu", g_sentPerSec,
           g_errPerSec);
  v.servoMask = g_servoMask;
  v.posValid = g_posValid;
  memcpy(v.pos, g_lastPos, sizeof(v.pos));
  v.warnMask = g_warnMask;
  ui::drawRun(v);
}

static void doCalibration() {
  ui::drawMessage("CALIBRATE", TFT_YELLOW, "Writing...");
  uint16_t res[SO101_NUM_JOINTS];
  const uint8_t ok = so101bus::calibrateCenter(res);
  so101bus::setTorque(false);
  for (auto& u : g_unwrap) u.reset();  // 値の基準が変わるので展開をやり直す

  char l1[22], l2[22], l3[22];
  snprintf(l1, sizeof(l1), "%s %d/%d", ok == SO101_ALL_JOINTS ? "OK" : "NG",
           __builtin_popcount(ok), SO101_NUM_JOINTS);
  snprintf(l2, sizeof(l2), "%4u %4u %4u", res[0], res[1], res[2]);
  snprintf(l3, sizeof(l3), "%4u %4u %4u", res[3], res[4], res[5]);
  showMessage("CALIBRATE", ok == SO101_ALL_JOINTS ? TFT_GREEN : TFT_RED, l1,
              l2, l3);
}

// ---------------------------------------------------------------------------
void setup() {
  ui::begin();
  ui::drawMessage("LEADER", TFT_ORANGE, "Starting...");

  g_servoMask = so101bus::begin();
  so101bus::setTorque(false);  // リーダーは手で動かすのでトルクOFF

  setupBle();
}

void loop() {
  static uint32_t lastSend = 0, lastStat = 0, lastDraw = 0;
  static uint32_t sentCount = 0;
  const uint32_t now = millis();
  const ui::Button btn = ui::poll();

  // ---- BLE イベント処理 ----
  if (g_pairRequested) {
    g_pairRequested = false;
    g_peer = g_pairAddr;
    g_hasPeer = true;
    peerstore::save(g_peer);
    showMessage("PAIRING", TFT_GREEN, "Paired with", g_peer.toString().c_str());
  }
  if (g_pendingHandle != kNoConn && now - g_pendingSince > kPendingTimeoutMs) {
    g_server->disconnect(g_pendingHandle);  // 要求を書かない相手は切る
    g_pendingSince = now;                   // 切断完了まで連打しない
  }
  if (g_needAdvUpdate) {
    g_needAdvUpdate = false;
    updateAdvertising();
  }

  // ---- 読み取り・送信 (RUN 中のみ) ----
  if (g_mode == Mode::Run && now - lastSend >= SO101_SEND_PERIOD_MS) {
    lastSend = now;
    JointPacket pkt;
    pkt.magic = SO101_PKT_MAGIC;
    pkt.version = SO101_PKT_VERSION;
    if (readLeaderJoints(pkt.pos) && g_connHandle != kNoConn) {
      pkt.seq = g_seq++;
      g_jointChr->setValue(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
      g_jointChr->notify();
      sentCount++;
    }
  }
  if (now - lastStat >= 1000) {
    lastStat = now;
    g_sentPerSec = sentCount;
    sentCount = 0;
    if (g_mode == Mode::Run) g_errPerSec = 0;
  }

  // ---- 画面・ボタン ----
  switch (g_mode) {
    case Mode::Run:
      if (btn == ui::Button::Hold) {
        g_mode = Mode::Menu;
        g_menuSel = 0;
        ui::drawMenu("MENU", kMenuItems, kMenuCount, g_menuSel);
      } else if (now - lastDraw >= 100) {
        lastDraw = now;
        drawRun();
      }
      break;

    case Mode::Menu:
      if (btn == ui::Button::Click) {
        g_menuSel = (g_menuSel + 1) % kMenuCount;
        ui::drawMenu("MENU", kMenuItems, kMenuCount, g_menuSel);
      } else if (btn == ui::Button::Hold) {
        if (g_menuSel == 0) {
          g_mode = Mode::CalibConfirm;
          ui::drawMessage("CALIBRATE", TFT_YELLOW, "Set arm to", "base pose.",
                          nullptr, nullptr, "hold:run click:cancel");
        } else if (g_menuSel == 1) {
          g_mode = Mode::Pairing;
          startPairing();
          ui::drawMessage("PAIRING", TFT_CYAN, "Waiting for", "follower...",
                          "(pair cleared)", nullptr, "click: cancel");
        } else {
          g_mode = Mode::Run;
        }
      }
      break;

    case Mode::CalibConfirm:
      if (btn == ui::Button::Hold) {
        doCalibration();
      } else if (btn == ui::Button::Click) {
        g_mode = Mode::Run;
      }
      break;

    case Mode::Pairing:
      // ペアリング完了は上の g_pairRequested で Message へ遷移
      if (btn == ui::Button::Click) {
        cancelPairing();
        showMessage("PAIRING", TFT_RED, "Cancelled.", "Not paired.");
      }
      break;

    case Mode::Message:
      if (btn == ui::Button::Click || (int32_t)(now - g_msgUntil) > 0) {
        g_mode = Mode::Run;
      }
      break;
  }
  delay(1);
}
