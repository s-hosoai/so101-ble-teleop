// SO101 サーボバス (STS3215 x6) アクセス層
// FEETECH 公式ライブラリ FTServo (SMS_STS) をラップする
#pragma once
#include <stdint.h>
#include "so101_link.h"

// ---- ハードウェア設定 -------------------------------------------------------
#ifndef SO101_BUS_TX_PIN
#define SO101_BUS_TX_PIN 7   // AtomS3 G7 -> サーボアダプタ RX
#endif
#ifndef SO101_BUS_RX_PIN
#define SO101_BUS_RX_PIN 8   // AtomS3 G8 <- サーボアダプタ TX
#endif
#ifndef SO101_BUS_BAUD
#define SO101_BUS_BAUD 1000000  // LeRobot 設定のデフォルト 1Mbps
#endif

// 関節順 (パケット内の並び) とサーボID
//   0:shoulder_pan 1:shoulder_lift 2:elbow_flex 3:wrist_flex 4:wrist_roll 5:gripper
static constexpr uint8_t SO101_SERVO_IDS[SO101_NUM_JOINTS] = {1, 2, 3, 4, 5, 6};
static constexpr uint8_t SO101_ALL_JOINTS = (1 << SO101_NUM_JOINTS) - 1;

// 校正後、基本姿勢で読める位置値
static constexpr uint16_t SO101_CENTER_POS = 2048;

namespace so101bus {

// UART 初期化と各サーボへの Ping。応答したサーボのビットマスクを返す
uint8_t begin();

// 全関節の現在位置を Sync Read で取得。読めた関節のビットマスクを返す
uint8_t readPositions(uint16_t pos[SO101_NUM_JOINTS]);

// 全関節へ目標位置を Sync Write (応答なし)
//   speed: 0 = 最大速度, acc: 0 = 加速度制限なし
void writePositions(const uint16_t pos[SO101_NUM_JOINTS], uint16_t speed,
                    uint8_t acc);

// 全関節のトルク ON/OFF
void setTorque(bool enable);

// EEPROM の角度リミットを読む。読めなかった関節は 0..4095。成功マスクを返す
uint8_t readLimits(uint16_t minPos[SO101_NUM_JOINTS],
                   uint16_t maxPos[SO101_NUM_JOINTS]);

// キャリブレーション:
//   現在の姿勢 (基本姿勢) が SO101_CENTER_POS と読めるよう、各サーボの
//   EEPROM の位置オフセット (Ofs) を書き換える。角度リミットも同じ量だけずらす。
//   トルクは OFF にする。成功した関節のビットマスクを返す。
//   resultPos には校正後に読み直した位置が入る。
uint8_t calibrateCenter(uint16_t resultPos[SO101_NUM_JOINTS]);

}  // namespace so101bus
