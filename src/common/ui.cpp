#include "ui.h"

#include "so101_bus.h"

namespace ui {

static M5Canvas g_canvas(&M5.Display);
static constexpr int kW = 128;
static constexpr int kLine = 10;
static const char* const kJointNames[SO101_NUM_JOINTS] = {"PAN", "LFT", "ELB",
                                                          "WFX", "WRL", "GRP"};

void begin() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setBrightness(80);
  M5.BtnA.setHoldThresh(800);
  g_canvas.setColorDepth(16);
  g_canvas.createSprite(M5.Display.width(), M5.Display.height());
  g_canvas.setFont(&fonts::Font0);
  g_canvas.setTextSize(1);
}

Button poll() {
  M5.update();
  if (M5.BtnA.wasHold()) return Button::Hold;
  if (M5.BtnA.wasClicked()) return Button::Click;
  return Button::None;
}

static void header(const char* title, uint16_t color) {
  g_canvas.fillRect(0, 0, kW, 12, color);
  g_canvas.setTextColor(TFT_BLACK);
  g_canvas.setTextDatum(top_center);
  g_canvas.drawString(title, kW / 2, 2);
  g_canvas.setTextDatum(top_left);
}

static void hintLine(const char* hint) {
  g_canvas.setTextColor(TFT_DARKGREY);
  g_canvas.setTextDatum(bottom_center);
  g_canvas.drawString(hint, kW / 2, 127);
  g_canvas.setTextDatum(top_left);
}

void drawRun(const RunView& v) {
  g_canvas.fillScreen(TFT_BLACK);
  header(v.role, v.roleColor);

  int y = 15;
  g_canvas.setTextColor(v.linkColor);
  g_canvas.drawString(v.link, 2, y);
  y += kLine;
  g_canvas.setTextColor(TFT_LIGHTGREY);
  g_canvas.drawString(v.peer, 2, y);
  y += kLine;
  g_canvas.drawString(v.stat, 2, y);
  y += kLine;

  // サーボ応答状況 "SV 123456" (無応答は赤)
  g_canvas.setTextColor(TFT_LIGHTGREY);
  g_canvas.drawString("SV", 2, y);
  for (int i = 0; i < SO101_NUM_JOINTS; i++) {
    const bool ok = v.servoMask & (1 << i);
    g_canvas.setTextColor(ok ? TFT_GREEN : TFT_RED);
    g_canvas.drawChar('0' + SO101_SERVO_IDS[i], 20 + i * 8, y);
  }
  y += kLine + 2;

  // 関節バー (中央線 = 2048)
  constexpr int barX = 50, barW = 76;
  for (int i = 0; i < SO101_NUM_JOINTS; i++) {
    const bool warn = v.warnMask & (1 << i);
    g_canvas.setTextColor(warn ? TFT_RED : TFT_WHITE);
    char buf[12];
    if (v.posValid) {
      snprintf(buf, sizeof(buf), "%s%5u", kJointNames[i], v.pos[i]);
    } else {
      snprintf(buf, sizeof(buf), "%s  ---", kJointNames[i]);
    }
    g_canvas.drawString(buf, 2, y);
    g_canvas.drawRect(barX, y, barW, 8, TFT_DARKGREY);
    if (v.posValid) {
      const int w = (int)v.pos[i] * (barW - 2) / 4095;
      g_canvas.fillRect(barX + 1, y + 1, w, 6, warn ? TFT_RED : TFT_SKYBLUE);
    }
    g_canvas.drawFastVLine(barX + barW / 2, y, 8, TFT_YELLOW);
    y += kLine;
  }

  hintLine("hold: menu");
  g_canvas.pushSprite(0, 0);
}

void drawMenu(const char* title, const char* const* items, int n, int sel) {
  g_canvas.fillScreen(TFT_BLACK);
  header(title, TFT_WHITE);
  int y = 20;
  for (int i = 0; i < n; i++) {
    if (i == sel) {
      g_canvas.fillRect(0, y - 2, kW, 14, TFT_BLUE);
      g_canvas.setTextColor(TFT_WHITE);
      g_canvas.drawString(">", 4, y + 1);
    } else {
      g_canvas.setTextColor(TFT_LIGHTGREY);
    }
    g_canvas.drawString(items[i], 14, y + 1);
    y += 16;
  }
  hintLine("click:next hold:OK");
  g_canvas.pushSprite(0, 0);
}

void drawMessage(const char* title, uint16_t titleColor, const char* l1,
                 const char* l2, const char* l3, const char* l4,
                 const char* hint) {
  g_canvas.fillScreen(TFT_BLACK);
  header(title, titleColor);
  g_canvas.setTextColor(TFT_WHITE);
  const char* lines[] = {l1, l2, l3, l4};
  int y = 24;
  for (const char* l : lines) {
    if (l) g_canvas.drawString(l, 2, y);
    y += 14;
  }
  if (hint) hintLine(hint);
  g_canvas.pushSprite(0, 0);
}

}  // namespace ui
