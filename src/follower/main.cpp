// SO101 フォロワー側: リーダーから BLE で関節角を受信しサーボへ反映する
// BLE セントラル (GATT クライアント) として動作
#include <Arduino.h>
#include <NimBLEDevice.h>

#include "../common/peer_store.h"
#include "../common/so101_bus.h"
#include "../common/so101_motion.h"
#include "../common/ui.h"
#include "so101_link.h"

// ---- 追従設定 ---------------------------------------------------------------
// 1 パケット (10ms) あたりの最大移動量 [step] (4096step = 1回転)
static constexpr uint16_t kMaxStep = 200;   // 通常時 (~1760deg/s)
static constexpr uint16_t kSoftStep = 8;    // 接続直後に寄せる時 (~70deg/s)
static constexpr uint8_t kAcc = 0;          // 0 = 加速度制限なし

// ---- 画面モード -------------------------------------------------------------
enum class Mode { Run, Menu, CalibConfirm, Pairing, Message };
static Mode g_mode = Mode::Run;
static int g_menuSel = 0;
static uint32_t g_msgUntil = 0;
static const char* const kMenuItems[] = {"Calibrate", "Pairing", "Back"};
static constexpr int kMenuCount = sizeof(kMenuItems) / sizeof(kMenuItems[0]);

// ---- 受信データ (NimBLE タスク → loop へ受け渡し) ---------------------------
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static JointPacket g_latest;
static volatile bool g_hasNew = false;
static volatile uint32_t g_lastRxMs = 0;
static volatile uint32_t g_rxCount = 0;
static volatile uint32_t g_lostCount = 0;
static volatile bool g_firstPkt = true;

// ---- 接続状態 ---------------------------------------------------------------
static NimBLEClient* g_client = nullptr;
static bool g_hasPeer = false;
static NimBLEAddress g_peer;
static NimBLEAddress g_targetAddr;
static volatile bool g_targetFound = false;
static volatile bool g_connected = false;
static volatile bool g_pairing = false;

// ---- サーボ -----------------------------------------------------------------
static uint8_t g_servoMask = 0;
static uint16_t g_minPos[SO101_NUM_JOINTS];
static uint16_t g_maxPos[SO101_NUM_JOINTS];
static SlewLimiter g_slew[SO101_NUM_JOINTS];
static uint16_t g_goal[SO101_NUM_JOINTS];
static bool g_following = false;   // トルクONで追従中
static bool g_softStart = false;
static bool g_linkAlive = false;
static uint8_t g_warnMask = 0;
static uint32_t g_rxPerSec = 0;

// ---------------------------------------------------------------------------
// 追従
// ---------------------------------------------------------------------------
// 追従開始: 現在位置を目標にしてからトルクONし、そこから徐々にリーダーへ寄せる
static void beginFollow(const uint16_t target[SO101_NUM_JOINTS]) {
  uint16_t cur[SO101_NUM_JOINTS];
  const uint8_t mask = so101bus::readPositions(cur);
  for (int i = 0; i < SO101_NUM_JOINTS; i++) {
    if (!(mask & (1 << i))) cur[i] = target[i];  // 読めなければ目標値から
    cur[i] = constrain(cur[i], g_minPos[i], g_maxPos[i]);
    g_slew[i].reset(cur[i]);
  }
  so101bus::writePositions(cur, 0, kAcc);  // トルクON時に古い目標へ飛ばないように
  so101bus::setTorque(true);
  g_following = true;
  g_softStart = true;
}

static void applyFollowerJoints(const uint16_t pos[SO101_NUM_JOINTS]) {
  if (!g_following) beginFollow(pos);

  const uint16_t step = g_softStart ? kSoftStep : kMaxStep;
  bool converged = true;
  g_warnMask = 0;
  for (int i = 0; i < SO101_NUM_JOINTS; i++) {
    const uint16_t target = constrain(pos[i], g_minPos[i], g_maxPos[i]);
    g_goal[i] = g_slew[i].step(target, step);
    const int err = abs((int)target - (int)g_goal[i]);
    if (err > kSoftStep) converged = false;
    // リミットでクランプ中 / 急変を制限中の関節は赤表示
    if (target != pos[i] || err > 0) g_warnMask |= 1 << i;
  }
  if (g_softStart && converged) g_softStart = false;
  so101bus::writePositions(g_goal, 0, kAcc);
}

// 通信途絶時: 最後の目標位置でトルクを保持したまま停止
// (脱力させたい場合は so101bus::setTorque(false) と g_following=false に変更)
static void onLinkLost() {}

static void stopFollow(bool torqueOff) {
  g_linkAlive = false;
  g_softStart = true;  // 再開時はゆっくり寄せる
  if (torqueOff) {
    so101bus::setTorque(false);
    g_following = false;
  }
}

// ---------------------------------------------------------------------------
// BLE
// ---------------------------------------------------------------------------
static void notifyCallback(NimBLERemoteCharacteristic* chr, uint8_t* data,
                           size_t len, bool isNotify) {
  if (len != sizeof(JointPacket)) return;
  JointPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.magic != SO101_PKT_MAGIC || pkt.version != SO101_PKT_VERSION) return;

  static uint16_t prevSeq = 0;
  if (!g_firstPkt) {
    const uint16_t gap = (uint16_t)(pkt.seq - prevSeq);
    if (gap > 1 && gap < 1000) g_lostCount += gap - 1;
  }
  g_firstPkt = false;
  prevSeq = pkt.seq;

  portENTER_CRITICAL(&g_mux);
  g_latest = pkt;
  g_hasNew = true;
  portEXIT_CRITICAL(&g_mux);
  g_lastRxMs = millis();
  g_rxCount++;
}

class ClientCallbacks : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient* client) override { g_connected = false; }
};

static bool isPairingAdv(NimBLEAdvertisedDevice* dev) {
  if (!dev->haveManufacturerData()) return false;
  const std::string d = dev->getManufacturerData();
  return d.size() >= SO101_ADV_LEN && (uint8_t)d[2] == SO101_ADV_MAGIC0 &&
         (uint8_t)d[3] == SO101_ADV_MAGIC1 &&
         (uint8_t)d[4] == SO101_ADV_MODE_PAIRING &&
         dev->isAdvertisingService(NimBLEUUID(SO101_SERVICE_UUID));
}

class ScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* dev) override {
    if (g_targetFound) return;
    const bool match = g_pairing ? isPairingAdv(dev)
                                 : (g_hasPeer && dev->getAddress() == g_peer);
    if (!match) return;
    g_targetAddr = dev->getAddress();
    g_targetFound = true;
    NimBLEDevice::getScan()->stop();
  }
};

static void startScan() {
  g_targetFound = false;
  NimBLEDevice::getScan()->start(0, nullptr, false);  // 見つかるまで無期限
}

static void stopScan() {
  NimBLEDevice::getScan()->stop();
  g_targetFound = false;
}

// 接続して購読する。ペアリング中なら先にペア要求を書き込み、成功したら保存
static bool connectToLeader() {
  if (g_client == nullptr) {
    g_client = NimBLEDevice::createClient();
    g_client->setClientCallbacks(new ClientCallbacks(), false);
    g_client->setConnectionParams(SO101_CONN_INTERVAL_MIN,
                                  SO101_CONN_INTERVAL_MAX, SO101_CONN_LATENCY,
                                  SO101_SUPERVISION_TIMEOUT);
    g_client->setConnectTimeout(3);
  }
  if (!g_client->connect(g_targetAddr)) return false;

  NimBLERemoteService* svc = g_client->getService(SO101_SERVICE_UUID);
  NimBLERemoteCharacteristic* joint =
      svc ? svc->getCharacteristic(SO101_JOINT_CHAR_UUID) : nullptr;
  NimBLERemoteCharacteristic* ctrl =
      svc ? svc->getCharacteristic(SO101_CTRL_CHAR_UUID) : nullptr;
  if (joint == nullptr || ctrl == nullptr || !joint->canNotify()) {
    g_client->disconnect();
    return false;
  }

  if (g_pairing) {
    const uint8_t req = SO101_CTRL_PAIR_REQ;
    if (!ctrl->writeValue(&req, 1, true)) {
      g_client->disconnect();
      return false;
    }
    g_pairing = false;
    g_peer = g_targetAddr;
    g_hasPeer = true;
    peerstore::save(g_peer);
  }

  g_firstPkt = true;
  if (!joint->subscribe(true, notifyCallback)) {
    g_client->disconnect();
    return false;
  }
  g_connected = true;
  return true;
}

static void disconnectLeader() {
  if (g_client && g_client->isConnected()) g_client->disconnect();
}

static void setupBle() {
  NimBLEDevice::init(SO101_FOLLOWER_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new ScanCallbacks(), true);
  scan->setDuplicateFilter(false);
  scan->setMaxResults(0);  // 結果は保持せずコールバックのみ
  scan->setActiveScan(true);
  scan->setInterval(45);
  scan->setWindow(30);

  g_hasPeer = peerstore::load(g_peer);
}

static void startPairing() {
  peerstore::clear();  // ペアリングのたびにクリア
  g_hasPeer = false;
  stopFollow(false);
  disconnectLeader();
  stopScan();
  g_pairing = true;
}

static void cancelPairing() {
  g_pairing = false;
  stopScan();
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
  v.role = "FOLLOWER";
  v.roleColor = TFT_CYAN;
  if (!g_hasPeer) {
    v.link = "NOT PAIRED";
    v.linkColor = TFT_DARKGREY;
  } else if (!g_connected) {
    v.link = g_targetFound ? "CONNECTING..." : "SEARCHING LEADER";
    v.linkColor = TFT_YELLOW;
  } else if (!g_linkAlive) {
    v.link = g_following ? "NO DATA (HOLD)" : "NO DATA";
    v.linkColor = TFT_ORANGE;
  } else {
    v.link = g_softStart ? "FOLLOWING (SOFT)" : "FOLLOWING";
    v.linkColor = TFT_GREEN;
  }
  snprintf(v.peer, sizeof(v.peer), "%s",
           g_hasPeer ? g_peer.toString().c_str() : "-");
  snprintf(v.stat, sizeof(v.stat), "rx %3lu/s lost %lu", g_rxPerSec,
           g_lostCount);
  v.servoMask = g_servoMask;
  v.posValid = g_following;
  memcpy(v.pos, g_goal, sizeof(v.pos));
  v.warnMask = g_warnMask;
  ui::drawRun(v);
}

static void doCalibration() {
  ui::drawMessage("CALIBRATE", TFT_YELLOW, "Writing...");
  uint16_t res[SO101_NUM_JOINTS];
  const uint8_t ok = so101bus::calibrateCenter(res);
  so101bus::setTorque(false);
  so101bus::readLimits(g_minPos, g_maxPos);  // リミットもずれたので読み直す

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
  ui::drawMessage("FOLLOWER", TFT_CYAN, "Starting...");

  g_servoMask = so101bus::begin();
  so101bus::readLimits(g_minPos, g_maxPos);
  so101bus::setTorque(false);  // 最初のパケットを受信するまでは脱力

  setupBle();
}

void loop() {
  static uint32_t lastStat = 0, lastDraw = 0;
  const uint32_t now = millis();
  const ui::Button btn = ui::poll();

  // ---- 接続管理 (RUN / PAIRING 中) ----
  if ((g_mode == Mode::Run || g_mode == Mode::Pairing) && !g_connected &&
      (g_pairing || g_hasPeer)) {
    if (g_targetFound) {
      if (g_mode == Mode::Run) drawRun();  // "CONNECTING..." を表示
      const bool wasPairing = g_pairing;
      if (connectToLeader()) {
        if (wasPairing) {
          showMessage("PAIRING", TFT_GREEN, "Paired with",
                      g_peer.toString().c_str());
        }
      } else {
        g_targetFound = false;  // 次のループで再スキャン
      }
    } else if (!NimBLEDevice::getScan()->isScanning()) {
      startScan();
    }
  }

  // ---- 受信データ反映 (RUN 中のみ) ----
  if (g_hasNew) {
    JointPacket pkt;
    portENTER_CRITICAL(&g_mux);
    pkt = g_latest;
    g_hasNew = false;
    portEXIT_CRITICAL(&g_mux);
    if (g_mode == Mode::Run) {
      applyFollowerJoints(pkt.pos);
      g_linkAlive = true;
    }
  }

  // ---- フェイルセーフ ----
  // g_lastRxMs は BLE タスクが更新するので now より新しい場合がある → 符号付きで比較
  if (g_linkAlive &&
      (int32_t)(now - g_lastRxMs) > (int32_t)SO101_RX_TIMEOUT_MS) {
    g_linkAlive = false;
    g_softStart = true;  // 復帰時はゆっくり寄せる
    onLinkLost();
  }

  if (now - lastStat >= 1000) {
    lastStat = now;
    g_rxPerSec = g_rxCount;
    g_rxCount = 0;
  }

  // ---- 画面・ボタン ----
  switch (g_mode) {
    case Mode::Run:
      if (btn == ui::Button::Hold) {
        stopFollow(false);  // メニュー中は現在位置で保持
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
          stopFollow(true);
          ui::drawMessage("CALIBRATE", TFT_YELLOW, "TORQUE OFF!",
                          "Support arm and", "set base pose.", nullptr,
                          "hold:run click:cancel");
        } else if (g_menuSel == 1) {
          g_mode = Mode::Pairing;
          startPairing();
          ui::drawMessage("PAIRING", TFT_CYAN, "Searching", "leader...",
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
      // ペアリング完了は接続管理で Message へ遷移
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
