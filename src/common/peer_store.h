// ペアリング相手 (1 台) の BLE アドレスを不揮発メモリ (NVS) に保存する
#pragma once
#include <NimBLEAddress.h>

namespace peerstore {

bool load(NimBLEAddress& out);  // 保存済みなら true
void save(const NimBLEAddress& addr);
void clear();

}  // namespace peerstore
