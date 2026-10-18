// CraneClaw の調整値を探すスケッチ。シリアルモニタ(115200bps)から 1 文字ずつ送って、サーボを少しずつ動かす。
//   l / r / b : 動かす側を選ぶ(左・右・両方)
//   + / -     : 10µs ずつ動かす      ] / [ : 100µs ずつ動かす
//   p         : 今の値を表示する
// 爪を手で触らず、目で見ながら「開いた角度」「閉じきる角度」「最大の角度」を探し、表示された値を自分のスケッチに書き写す。
// 手順は README.md の「調整値の決め方」。

#include <CraneClaw.h>

const uint8_t LEFT_PIN = 22;
const uint8_t RIGHT_PIN = 23;
const uint16_t START_US = 1450;  // 最初の位置(中立付近)

CraneClaw claw;
uint16_t leftUs = START_US;
uint16_t rightUs = START_US;
char side = 'b';  // 'l'・'r'・'b'

void printValues() {
    Serial.print("left=");
    Serial.print(leftUs);
    Serial.print(" us  right=");
    Serial.print(rightUs);
    Serial.print(" us  (moving: ");
    Serial.print(side == 'l' ? "left" : side == 'r' ? "right" : "both");
    Serial.println(")");
}

void step(int delta) {
    if (side == 'l' || side == 'b') leftUs = constrain(leftUs + delta, CRANE_CLAW_PULSE_MIN_US, CRANE_CLAW_PULSE_MAX_US);
    if (side == 'r' || side == 'b') rightUs = constrain(rightUs + delta, CRANE_CLAW_PULSE_MIN_US, CRANE_CLAW_PULSE_MAX_US);
    claw.moveToPulse(leftUs, rightUs);
    printValues();
}

void setup() {
    Serial.begin(115200);
    CraneClawConfig config = {
        {START_US, START_US, START_US},
        {START_US, START_US, START_US},
        0, 0, 0,
        300, 300, 300,  // moveToPulse は「閉じる時間」(300ms)をかけて動く
    };
    claw.begin(LEFT_PIN, RIGHT_PIN, config);
    claw.signalOn();  // 両方とも START_US から始まる
}

void loop() {
    claw.loop();
    if (Serial.available() <= 0) return;
    char c = Serial.read();
    switch (c) {
        case 'l': case 'r': case 'b': side = c; printValues(); break;
        case '+': step(10); break;
        case '-': step(-10); break;
        case ']': step(100); break;
        case '[': step(-100); break;
        case 'p': printValues(); break;
        default: break;
    }
}
