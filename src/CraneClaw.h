// CraneClaw: クレーンゲームの爪(サーボ 2 個)を動かす Arduino ライブラリ
// https://github.com/eke2537/CraneClaw  MIT ライセンス(LICENSE)
//
// 爪をばね越しにサーボで閉じる機構のためのライブラリ。サーボを「爪が閉じきる角度」より先へ進めると
// ばねが巻かれてつかむ力が強くなる。つかむ力(0〜100)は、このばねの巻き量として指定する。
// 使い方と解説は README.md。
//
// 使い方(要点)
//   1. setup() で begin() を 1 回呼ぶ
//   2. 爪を動かす間だけ signalOn()、終わったら signalOff()
//   3. openArm() / closeArm() / restArm() で開閉する(呼んだらすぐ戻る)
//   4. loop() の中で毎回 claw.loop() を呼ぶ(これを呼ばないと爪が動かない)

#ifndef CRANE_CLAW_H
#define CRANE_CLAW_H

#include <Arduino.h>
#include <Servo.h>

// 出してよいパルス幅の範囲(マイクロ秒)。指定した値はこの範囲に丸める
#define CRANE_CLAW_PULSE_MIN_US 500
#define CRANE_CLAW_PULSE_MAX_US 2400

// 片側の爪の調整値(サーボのパルス幅、マイクロ秒)。左右でサーボの向きが逆でもよい(大小は問わない)
struct CraneClawAngles {
    uint16_t openArm;   // 開いた角度
    uint16_t closeArm;  // 爪がちょうど閉じきる角度(つかむ力 0)。信号を出し始めたときもこの角度
    uint16_t maxArm;    // 最大の角度(つかむ力 100)。ばねが巻き切られる手前、サーボの可動範囲の手前に置く
};

// 設定値
struct CraneClawConfig {
    CraneClawAngles left;    // 左の爪
    CraneClawAngles right;   // 右の爪
    uint8_t gripPower;       // つかむ力 0〜100
    uint8_t carryPower;      // 運ぶ力 0〜100(弱めが当たったときに緩める先)
    uint8_t weakenPercent;   // 弱める確率 0〜100(%)
    uint32_t openTimeMs;     // 開くのにかける時間(ミリ秒。0 ならすぐ)
    uint32_t closeTimeMs;    // 閉じるのにかける時間
    uint32_t weakenTimeMs;   // 弱めるのにかける時間
};

class CraneClaw {
public:
    // 最初に 1 回呼ぶ。左右のサーボの端子と設定を受け取る。信号はまだ出さない(端子は L)
    void begin(uint8_t leftPin, uint8_t rightPin, const CraneClawConfig &config);

    // サーボに信号を出し始める。左右とも「爪が閉じきる角度」から始める
    void signalOn();
    // 信号を止める。出しているパルスを出し終えてから止まり、端子は L になる
    void signalOff();
    bool isSignalOn() const { return signalOn_; }

    // 爪を開く / つかむ力で閉じる / 爪が閉じきる角度(つかむ力 0)まで閉じる。信号を止めている間は何もしない
    void openArm();
    void closeArm();  // 弱めの状態を戻す
    void restArm();   // プレイの終わりに使う。ばねを巻いたまま信号を止めないため

    // つかむ力を変える(0〜100)。閉じていて弱めていないときは、閉じる時間をかけてその場で反映する
    void setGripPower(uint8_t power);

    // 確率で弱める。閉じていて、まだ弱めていないときだけ判定し、当たれば運ぶ力へ緩めて true を返す
    bool maybeWeaken();

    // 調整用。左右のパルス幅を直接指定し、閉じる時間をかけて動かす
    void moveToPulse(uint16_t leftUs, uint16_t rightUs);

    // 設定を差し替える。信号と今の位置はそのまま(次に動かすときから効く)
    void updateConfig(const CraneClawConfig &config);
    const CraneClawConfig &config() const { return cfg_; }

    // loop() の中で毎回呼ぶ。20ms ごとに指令を少しずつ動かす
    void loop();

    bool isMoving() const { return moving_; }          // 動いている途中なら true
    bool wasWeakened() const { return weakened_; }     // 閉じてから弱めが当たったら true
    // 今の指令(パルス幅、マイクロ秒)。信号を止めている間は 0
    void getPosition(uint16_t &leftUs, uint16_t &rightUs) const;

private:
    enum Pose { POSE_REST, POSE_OPEN, POSE_CLOSED };

    struct Side {
        int32_t fromUs;
        int32_t toUs;
        int32_t nowUs;
    };

    void storeConfig(const CraneClawConfig &config);
    int32_t powerToUs(const CraneClawAngles &angles, uint8_t power) const;
    void startMove(int32_t leftTo, int32_t rightTo, uint32_t ms);
    void moveToPower(uint8_t power, uint32_t ms);
    void writeOutputs();

    CraneClawConfig cfg_ = {};
    uint8_t leftPin_ = 0;
    uint8_t rightPin_ = 0;
    Servo leftServo_;
    Servo rightServo_;
    bool signalOn_ = false;
    Pose pose_ = POSE_REST;
    bool weakened_ = false;
    Side left_ = {};
    Side right_ = {};
    uint32_t moveStartUs_ = 0;
    uint32_t moveMs_ = 0;
    uint32_t lastUpdateUs_ = 0;
    bool moving_ = false;
};

#endif
