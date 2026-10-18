// CraneClaw の基本の動き: 閉じる → 弱めの判定 → 開く を繰り返し、爪の位置をシリアルモニタに出す。
// ピン(左 GPIO22・右 GPIO23)と調整値は仮の値。自分の機体に合わせて直す。
// 調整値の探し方は examples/Calibration と README.md の「調整値の決め方」。

#include <CraneClaw.h>

const uint8_t LEFT_PIN = 22;
const uint8_t RIGHT_PIN = 23;

CraneClaw claw;

const CraneClawConfig CONFIG = {
    {1133, 1450, 1450},  // 左: 開いた角度・閉じきる角度・最大の角度(µs)
    {1767, 1450, 1450},  // 右(左と逆向きに回る前提)
    80,                  // つかむ力
    30,                  // 運ぶ力
    30,                  // 弱める確率(%)
    800,                 // 開く時間(ms)
    800,                 // 閉じる時間(ms)
    500,                 // 弱める時間(ms)
};

// 指定の時間、claw.loop() を呼び続けて待つ
void waitMs(uint32_t ms) {
    uint32_t start = millis();
    while (millis() - start < ms) {
        claw.loop();
    }
}

void printPosition(const char *label) {
    uint16_t l, r;
    claw.getPosition(l, r);
    Serial.print(label);
    Serial.print(" left=");
    Serial.print(l);
    Serial.print(" right=");
    Serial.print(r);
    Serial.print(" weakened=");
    Serial.println(claw.wasWeakened());
}

void setup() {
    Serial.begin(115200);
    // 弱めの判定の乱数を起動のたびに変える
#if defined(ARDUINO_ARCH_RP2040)
    randomSeed(rp2040.hwrand32());
#else
    randomSeed(analogRead(A0));  // 何もつないでいないアナログ端子の雑音を種にする
#endif
    claw.begin(LEFT_PIN, RIGHT_PIN, CONFIG);
    delay(2000);
    claw.signalOn();
    printPosition("signalOn");
}

void loop() {
    claw.closeArm();
    waitMs(1500);
    printPosition("close");

    claw.maybeWeaken();
    waitMs(1000);
    printPosition("weaken");

    claw.openArm();
    waitMs(1500);
    printPosition("open");
}
