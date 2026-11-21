#include "so101_bus.h"

#include <Arduino.h>
#include <SCServo.h>

namespace so101bus {

static SMS_STS g_sts;
static constexpr uint8_t kPosLen = 2;  // Present_Position は 2byte
static constexpr uint32_t kSyncReadTimeoutMs = 3;
static constexpr int kOfsSignBit = 11;  // Ofs は bit11 が符号 (符号+絶対値)
static constexpr int kOfsMax = 2047;

static int decodeSignMag(int w, int bit) {
  const int mag = w & ((1 << bit) - 1);
  return (w & (1 << bit)) ? -mag : mag;
}
static uint16_t encodeSignMag(int v, int bit) {
  return v < 0 ? (uint16_t)((-v) | (1 << bit)) : (uint16_t)v;
}
static int mod4096(int v) { return ((v % 4096) + 4096) % 4096; }

uint8_t begin() {
  Serial1.setRxBufferSize(256);
  Serial1.begin(SO101_BUS_BAUD, SERIAL_8N1, SO101_BUS_RX_PIN, SO101_BUS_TX_PIN);
  Serial1.setRxTimeout(1);  // 受信をすぐリングバッファへ上げる (低遅延化)
  g_sts.pSerial = &Serial1;
  g_sts.IOTimeOut = 3;
  delay(50);

  // Sync Read 用の受信バッファを確保 (以後ずっと使い回す)
  g_sts.syncReadBegin(SO101_NUM_JOINTS, kPosLen, kSyncReadTimeoutMs);

  uint8_t mask = 0;
  for (int i = 0; i < SO101_NUM_JOINTS; i++) {
    const uint8_t id = SO101_SERVO_IDS[i];
    if (g_sts.Ping(id) == id) mask |= 1 << i;
  }
  return mask;
}

uint8_t readPositions(uint16_t pos[SO101_NUM_JOINTS]) {
  uint8_t ids[SO101_NUM_JOINTS];
  memcpy(ids, SO101_SERVO_IDS, sizeof(ids));
  g_sts.syncReadPacketTx(ids, SO101_NUM_JOINTS, SMS_STS_PRESENT_POSITION_L,
                         kPosLen);

  uint8_t mask = 0;
  uint8_t buf[kPosLen];
  for (int i = 0; i < SO101_NUM_JOINTS; i++) {
    if (!g_sts.syncReadPacketRx(ids[i], buf)) continue;
    const int v = g_sts.syncReadRxPacketToWrod(15);  // bit15 は符号ビット
    if (v < 0 || v > 4095) continue;
    pos[i] = (uint16_t)v;
    mask |= 1 << i;
  }
  return mask;
}

void writePositions(const uint16_t pos[SO101_NUM_JOINTS], uint16_t speed,
                    uint8_t acc) {
  uint8_t ids[SO101_NUM_JOINTS];
  s16 p[SO101_NUM_JOINTS];  // ライブラリが配列を書き換えるのでコピーを渡す
  u16 spd[SO101_NUM_JOINTS];
  u8 ac[SO101_NUM_JOINTS];
  for (int i = 0; i < SO101_NUM_JOINTS; i++) {
    ids[i] = SO101_SERVO_IDS[i];
    p[i] = (s16)pos[i];
    spd[i] = speed;
    ac[i] = acc;
  }
  g_sts.SyncWritePosEx(ids, SO101_NUM_JOINTS, p, spd, ac);
}

void setTorque(bool enable) {
  for (int i = 0; i < SO101_NUM_JOINTS; i++) {
    g_sts.EnableTorque(SO101_SERVO_IDS[i], enable ? 1 : 0);
  }
}

uint8_t readLimits(uint16_t minPos[SO101_NUM_JOINTS],
                   uint16_t maxPos[SO101_NUM_JOINTS]) {
  uint8_t mask = 0;
  for (int i = 0; i < SO101_NUM_JOINTS; i++) {
    const uint8_t id = SO101_SERVO_IDS[i];
    const int lo = g_sts.readWord(id, SMS_STS_MIN_ANGLE_LIMIT_L);
    const int hi = g_sts.readWord(id, SMS_STS_MAX_ANGLE_LIMIT_L);
    const bool ok = lo >= 0 && hi > lo && hi <= 4095;
    minPos[i] = ok ? lo : 0;
    maxPos[i] = ok ? hi : 4095;
    if (ok) mask |= 1 << i;
  }
  return mask;
}

// 1 サーボ分のキャリブレーション
static bool calibrateOne(uint8_t id, uint16_t& resultPos) {
  g_sts.EnableTorque(id, 0);

  const int ofsW = g_sts.readWord(id, SMS_STS_OFS_L);
  const int present = g_sts.readWord(id, SMS_STS_PRESENT_POSITION_L);
  const int lo = g_sts.readWord(id, SMS_STS_MIN_ANGLE_LIMIT_L);
  const int hi = g_sts.readWord(id, SMS_STS_MAX_ANGLE_LIMIT_L);
  if (ofsW < 0 || present < 0 || lo < 0 || hi < 0) return false;

  // Present = Raw - Ofs (mod 4096)
  const int oldOfs = decodeSignMag(ofsW, kOfsSignBit);
  const int raw = mod4096((present & 0x0FFF) + oldOfs);
  int newOfs = raw - SO101_CENTER_POS;  // -2048..2047
  if (newOfs < -kOfsMax) newOfs = -kOfsMax;
  const int shift = newOfs - oldOfs;  // 新 Present = 旧 Present - shift

  // 角度リミットも同じ物理範囲を保つようにずらす。
  // 0/0 (マルチターン) と 0..4095 (全域) はそのまま。はみ出す場合は全域にする。
  int newLo = lo, newHi = hi;
  const bool keep = (lo == 0 && hi == 0) || (lo == 0 && hi == 4095);
  if (!keep) {
    newLo = lo - shift;
    newHi = hi - shift;
    if (newLo < 0 || newHi > 4095) {
      newLo = 0;
      newHi = 4095;
    }
  }

  g_sts.unLockEprom(id);
  bool ok = g_sts.writeWord(id, SMS_STS_OFS_L, encodeSignMag(newOfs, kOfsSignBit));
  if (ok && !keep) {
    ok &= g_sts.writeWord(id, SMS_STS_MIN_ANGLE_LIMIT_L, newLo) == 1;
    ok &= g_sts.writeWord(id, SMS_STS_MAX_ANGLE_LIMIT_L, newHi) == 1;
  }
  g_sts.LockEprom(id);
  if (!ok) return false;

  delay(20);
  const int verify = g_sts.readWord(id, SMS_STS_PRESENT_POSITION_L);
  if (verify < 0) return false;
  resultPos = verify & 0x0FFF;
  return abs((int)resultPos - (int)SO101_CENTER_POS) <= 8;
}

uint8_t calibrateCenter(uint16_t resultPos[SO101_NUM_JOINTS]) {
  uint8_t mask = 0;
  for (int i = 0; i < SO101_NUM_JOINTS; i++) {
    resultPos[i] = 0;
    if (calibrateOne(SO101_SERVO_IDS[i], resultPos[i])) mask |= 1 << i;
  }
  return mask;
}

}  // namespace so101bus
