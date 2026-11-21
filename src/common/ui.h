// AtomS3 (128x128 LCD + 画面ボタン) 用の画面表示とメニュー
//   クリック: 次の項目 / 長押し: 決定
#pragma once
#include <M5Unified.h>
#include "so101_link.h"

namespace ui {

enum class Button { None, Click, Hold };

void begin();
Button poll();  // 毎ループ呼ぶ (M5.update を内部で実行)

// 通常画面 (RUN) の表示内容
struct RunView {
  const char* role;        // "LEADER" / "FOLLOWER"
  uint16_t roleColor;
  const char* link;        // 接続状態の文字列
  uint16_t linkColor;
  char peer[20];           // ペア相手アドレス
  char stat[22];           // 自由記述 (送受信レート等)
  uint8_t servoMask;       // 応答したサーボ
  bool posValid;
  uint16_t pos[SO101_NUM_JOINTS];
  uint8_t warnMask;        // 赤表示する関節 (跨ぎ飽和中など)
};
void drawRun(const RunView& v);

void drawMenu(const char* title, const char* const* items, int n, int sel);

// 最大 4 行のメッセージ画面
void drawMessage(const char* title, uint16_t titleColor, const char* l1,
                 const char* l2 = nullptr, const char* l3 = nullptr,
                 const char* l4 = nullptr, const char* hint = nullptr);

}  // namespace ui
