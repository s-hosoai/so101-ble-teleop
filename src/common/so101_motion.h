// 位置値の 4095<->0 跨ぎ対策と急変防止
#pragma once
#include <stdint.h>

static constexpr int32_t SO101_POS_RES = 4096;
static constexpr int32_t SO101_POS_MAX = SO101_POS_RES - 1;

// 差分を -2048..2047 に折り返す (最短方向の変化量)
inline int32_t so101WrapDelta(int32_t d) {
  d %= SO101_POS_RES;
  if (d >= SO101_POS_RES / 2) d -= SO101_POS_RES;
  if (d < -SO101_POS_RES / 2) d += SO101_POS_RES;
  return d;
}

// リーダー用: 生値の跨ぎを連続値に展開し、0..4095 で飽和させる。
// 跨いだ側では端に張り付くので、フォロワーが反対端へ一気に振れることはない。
class Unwrapper {
 public:
  void reset() { init_ = false; }
  uint16_t update(uint16_t raw) {
    if (!init_) {
      cont_ = raw;
      init_ = true;
    } else {
      cont_ += so101WrapDelta((int32_t)raw - prev_);
    }
    prev_ = raw;
    if (cont_ < 0) return 0;
    if (cont_ > SO101_POS_MAX) return SO101_POS_MAX;
    return (uint16_t)cont_;
  }
  // 範囲外 (跨いで飽和中) なら true
  bool saturated() const {
    return init_ && (cont_ < 0 || cont_ > SO101_POS_MAX);
  }

 private:
  bool init_ = false;
  int32_t cont_ = 0;
  uint16_t prev_ = 0;
};

// フォロワー用: 1 回の更新で動く量を制限する
class SlewLimiter {
 public:
  void reset(uint16_t cur) { cur_ = cur; }
  uint16_t step(uint16_t target, uint16_t maxStep) {
    int32_t d = (int32_t)target - cur_;
    if (d > maxStep) d = maxStep;
    if (d < -(int32_t)maxStep) d = -(int32_t)maxStep;
    cur_ += d;
    return (uint16_t)cur_;
  }
  uint16_t value() const { return (uint16_t)cur_; }

 private:
  int32_t cur_ = 2048;
};
