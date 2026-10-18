#include "CraneClaw.h"

#include <math.h>

// サーボの周期。指令はこの間隔で 1 回だけ更新する(サーボはこれより細かい変化を読まない。ServoEasing と同じ)
static const uint32_t UPDATE_INTERVAL_US = 20000;

static uint16_t clampPulse(uint16_t us) {
    if (us < CRANE_CLAW_PULSE_MIN_US) return CRANE_CLAW_PULSE_MIN_US;
    if (us > CRANE_CLAW_PULSE_MAX_US) return CRANE_CLAW_PULSE_MAX_US;
    return us;
}

static uint8_t clamp100(uint8_t v) {
    return v > 100 ? 100 : v;
}

// 0〜1 の経過の割合を、始めと終わりの速さが 0 になる進み具合に変える。
// ServoEasing(ArminJo)の SINE の IN_OUT と同じ (1 − cos(πt)) / 2。途中の最高速度は一定の速さの場合の約 1.57 倍
static float easeSineInOut(float t) {
    return (1.0f - cosf((float)M_PI * t)) / 2.0f;
}

void CraneClaw::storeConfig(const CraneClawConfig &config) {
    cfg_ = config;
    CraneClawAngles *sides[] = {&cfg_.left, &cfg_.right};
    for (CraneClawAngles *a : sides) {
        a->openArm = clampPulse(a->openArm);
        a->closeArm = clampPulse(a->closeArm);
        a->maxArm = clampPulse(a->maxArm);
    }
    cfg_.gripPower = clamp100(cfg_.gripPower);
    cfg_.carryPower = clamp100(cfg_.carryPower);
    cfg_.weakenPercent = clamp100(cfg_.weakenPercent);
}

// つかむ力 0〜100 を「爪が閉じきる角度」〜「最大の角度」へ直線で割り当てる
int32_t CraneClaw::powerToUs(const CraneClawAngles &angles, uint8_t power) const {
    int32_t closed = angles.closeArm;
    int32_t span = (int32_t)angles.maxArm - closed;
    return closed + span * clamp100(power) / 100;
}

void CraneClaw::writeOutputs() {
    leftServo_.writeMicroseconds(left_.nowUs);
    rightServo_.writeMicroseconds(right_.nowUs);
}

void CraneClaw::startMove(int32_t leftTo, int32_t rightTo, uint32_t ms) {
    left_.fromUs = left_.nowUs;
    left_.toUs = leftTo;
    right_.fromUs = right_.nowUs;
    right_.toUs = rightTo;
    moveStartUs_ = micros();
    moveMs_ = ms;
    moving_ = true;
    // 次の更新は loop() で、前の更新から 20ms 経ったとき
}

void CraneClaw::moveToPower(uint8_t power, uint32_t ms) {
    startMove(powerToUs(cfg_.left, power), powerToUs(cfg_.right, power), ms);
}

void CraneClaw::begin(uint8_t leftPin, uint8_t rightPin, const CraneClawConfig &config) {
    storeConfig(config);
    leftPin_ = leftPin;
    rightPin_ = rightPin;
    // 信号を出すまでは端子を L にしておく
    pinMode(leftPin_, OUTPUT);
    digitalWrite(leftPin_, LOW);
    pinMode(rightPin_, OUTPUT);
    digitalWrite(rightPin_, LOW);
    signalOn_ = false;
    pose_ = POSE_REST;
    weakened_ = false;
    moving_ = false;
}

void CraneClaw::signalOn() {
    if (signalOn_) return;
    left_.nowUs = left_.fromUs = left_.toUs = cfg_.left.closeArm;
    right_.nowUs = right_.fromUs = right_.toUs = cfg_.right.closeArm;
    pose_ = POSE_REST;
    weakened_ = false;
    moving_ = false;
    signalOn_ = true;
    // 最初のパルスから「爪が閉じきる角度」を出す(始めの位置を決めずに出すと急に動く。ServoEasing の注意と同じ)
#if defined(ARDUINO_ARCH_RP2040)
    // arduino-pico の Servo は attach に初めの幅を渡せる
    leftServo_.attach(leftPin_, CRANE_CLAW_PULSE_MIN_US, CRANE_CLAW_PULSE_MAX_US, left_.nowUs);
    rightServo_.attach(rightPin_, CRANE_CLAW_PULSE_MIN_US, CRANE_CLAW_PULSE_MAX_US, right_.nowUs);
#else
    // 標準の Servo は attach の前に幅を書いておくと、その幅から始まる
    writeOutputs();
    leftServo_.attach(leftPin_, CRANE_CLAW_PULSE_MIN_US, CRANE_CLAW_PULSE_MAX_US);
    rightServo_.attach(rightPin_, CRANE_CLAW_PULSE_MIN_US, CRANE_CLAW_PULSE_MAX_US);
    writeOutputs();
#endif
    lastUpdateUs_ = micros();
}

void CraneClaw::signalOff() {
    signalOn_ = false;
    moving_ = false;
    leftServo_.detach();
    rightServo_.detach();
}

void CraneClaw::openArm() {
    if (!signalOn_) return;
    pose_ = POSE_OPEN;
    startMove(cfg_.left.openArm, cfg_.right.openArm, cfg_.openTimeMs);
}

void CraneClaw::closeArm() {
    if (!signalOn_) return;
    pose_ = POSE_CLOSED;
    weakened_ = false;
    moveToPower(cfg_.gripPower, cfg_.closeTimeMs);
}

void CraneClaw::restArm() {
    if (!signalOn_) return;
    pose_ = POSE_REST;
    moveToPower(0, cfg_.closeTimeMs);
}

void CraneClaw::setGripPower(uint8_t power) {
    cfg_.gripPower = clamp100(power);
    if (signalOn_ && pose_ == POSE_CLOSED && !weakened_) {
        moveToPower(cfg_.gripPower, cfg_.closeTimeMs);
    }
}

bool CraneClaw::maybeWeaken() {
    if (!signalOn_ || pose_ != POSE_CLOSED || weakened_) return false;
    if (random(100) >= cfg_.weakenPercent) return false;
    weakened_ = true;
    moveToPower(cfg_.carryPower, cfg_.weakenTimeMs);
    return true;
}

void CraneClaw::moveToPulse(uint16_t leftUs, uint16_t rightUs) {
    if (!signalOn_) return;
    pose_ = POSE_REST;
    startMove(clampPulse(leftUs), clampPulse(rightUs), cfg_.closeTimeMs);
}

void CraneClaw::updateConfig(const CraneClawConfig &config) {
    storeConfig(config);
}

void CraneClaw::loop() {
    if (!signalOn_ || !moving_) return;
    uint32_t now = micros();
    if (now - lastUpdateUs_ < UPDATE_INTERVAL_US) return;
    lastUpdateUs_ = now;

    uint32_t elapsedUs = now - moveStartUs_;
    uint32_t totalUs = moveMs_ * 1000;
    if (elapsedUs >= totalUs) {
        left_.nowUs = left_.toUs;
        right_.nowUs = right_.toUs;
        moving_ = false;
    } else {
        // 経過時間で割り当てるので、loop() を呼ぶ間隔がぶれても全体の時間は変わらない
        float s = easeSineInOut((float)elapsedUs / (float)totalUs);
        left_.nowUs = left_.fromUs + (int32_t)lroundf((float)(left_.toUs - left_.fromUs) * s);
        right_.nowUs = right_.fromUs + (int32_t)lroundf((float)(right_.toUs - right_.fromUs) * s);
    }
    writeOutputs();
}

void CraneClaw::getPosition(uint16_t &leftUs, uint16_t &rightUs) const {
    leftUs = signalOn_ ? (uint16_t)left_.nowUs : 0;
    rightUs = signalOn_ ? (uint16_t)right_.nowUs : 0;
}
