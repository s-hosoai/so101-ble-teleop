// リーダー/フォロワー共通の BLE リンク定義
#pragma once
#include <stdint.h>

// ---- BLE 識別子 -------------------------------------------------------------
#define SO101_LEADER_NAME "SO101-Leader"
#define SO101_FOLLOWER_NAME "SO101-Follower"

#define SO101_SERVICE_UUID "6f0a0001-5d1e-4c2b-9a2e-50f101a7e000"
#define SO101_JOINT_CHAR_UUID "6f0a0002-5d1e-4c2b-9a2e-50f101a7e000"  // Notify
#define SO101_CTRL_CHAR_UUID "6f0a0003-5d1e-4c2b-9a2e-50f101a7e000"   // Write

// ---- アドバタイズ (Manufacturer Data) ---------------------------------------
// [0..1] Company ID 0xFFFF (テスト用) / [2..3] 'S','1' / [4] モード
static constexpr uint8_t SO101_ADV_MAGIC0 = 'S';
static constexpr uint8_t SO101_ADV_MAGIC1 = '1';
static constexpr uint8_t SO101_ADV_MODE_NORMAL = 0;
static constexpr uint8_t SO101_ADV_MODE_PAIRING = 1;
static constexpr uint8_t SO101_ADV_LEN = 5;

// ---- ペアリング要求 (フォロワー → リーダーの CTRL 書き込み) -----------------
static constexpr uint8_t SO101_CTRL_PAIR_REQ = 0x50;

// ---- 接続パラメータ (単位 1.25ms) -------------------------------------------
static constexpr uint16_t SO101_CONN_INTERVAL_MIN = 6;     // 7.5ms
static constexpr uint16_t SO101_CONN_INTERVAL_MAX = 12;    // 15ms
static constexpr uint16_t SO101_CONN_LATENCY = 0;
static constexpr uint16_t SO101_SUPERVISION_TIMEOUT = 100; // 1s (10ms 単位)

// ---- 送信周期 / フェイルセーフ ----------------------------------------------
static constexpr uint32_t SO101_SEND_PERIOD_MS = 10;   // 100Hz
static constexpr uint32_t SO101_RX_TIMEOUT_MS = 200;   // これ以上途切れたら停止扱い

// ---- パケット ---------------------------------------------------------------
static constexpr uint8_t SO101_NUM_JOINTS = 6;  // shoulder_pan .. gripper
static constexpr uint8_t SO101_PKT_MAGIC = 0x5A;
static constexpr uint8_t SO101_PKT_VERSION = 1;

#pragma pack(push, 1)
struct JointPacket {
  uint8_t magic;                     // SO101_PKT_MAGIC
  uint8_t version;                   // SO101_PKT_VERSION
  uint16_t seq;                      // 連番 (欠損検出用)
  uint16_t pos[SO101_NUM_JOINTS];    // STS3215 位置値 0..4095 (校正後 基本姿勢=2048)
};
#pragma pack(pop)

static_assert(sizeof(JointPacket) <= 20, "JointPacket must fit in default MTU");
