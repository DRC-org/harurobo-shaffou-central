#include <Arduino.h>
#include <ps5Controller.h>
#include <cmath>
#include <cstring>
#include "driver/twai.h"

// =========================================================
// 通常運転用のメインプログラム
//
// このファイルは次の順番で読めるように並べてある。
//
// 1. 設定値
// 2. 低レベルの共通関数
// 3. ボタン入力処理
// 4. オムニ足回り計算
// 5. CAN受信処理
// 6. 初期化
// 7. loop()
//
// このリポジトリ内の他ファイルとの関係:
// - normal_main.cpp
//   普段の運転で使う本番用
// - dm_test_main.cpp
//   DM モータ 1 台だけを回して確認する用
// - dm_id_writer_main.cpp
//   DM モータの CAN ID を書き換える用
// =========================================================
//
// さらに処理の流れを具体的に書くと:
// - setup()
//   1. Serial を開始
//   2. PS5 接続の準備
//   3. CAN の開始
//   4. DM モータ 4 台を速度モードにして Enable
// - loop()
//   1. PS5 のボタン/スティックを読む
//   2. ボタンなら各機構へ CAN を送る
//   3. スティックなら 4 輪の目標速度を計算する
//   4. 4 輪の目標速度を DM モータへ送る
//   5. CAN 受信があれば状態表示する
//
// どこを触れば何が変わるか:
// - ボタン割り当て        → handleButtonInput()
// - 足回りの計算式        → updateOmniTargetsFromController()
// - 最高速度             → MAX_LINEAR_SPEED_M_S / MAX_ANGULAR_SPEED_RAD_S
// - 車輪速度の上限       → MAX_WHEEL_SPEED_RAD_S
// - DM モータの ID       → OMNI_MOTOR_CAN_IDS / OMNI_FEEDBACK_IDS

// =========================================================
// 1. 設定値
// =========================================================

// ESP32 と CAN トランシーバをつなぐピン
// 配線を変えたらここも直す
#define CAN_TX_PIN GPIO_NUM_16
#define CAN_RX_PIN GPIO_NUM_4

// シリアルモニタの通信速度
// Arduino IDE / PlatformIO monitor 側も同じ値にする
const uint32_t SERIAL_BAUD = 115200;

// PS5 コントローラの MAC アドレス
// 別のコントローラにしたらここを書き換える
const char PS5_CONTROLLER_MAC[] = "7c:66:ef:84:dc:16";

// loop() の各処理周期
// 数字を小さくすると反応は速くなるが、CPUの仕事は増える
const unsigned long CONTROLLER_READ_INTERVAL_MS = 10;
const unsigned long RX_INTERVAL_MS = 2;
const unsigned long SPEED_SEND_INTERVAL_MS = 50;

// 最後に処理した時刻
// millis() と比較して「そろそろ次をやるか」を決める
unsigned long last_controller_read_ms = 0;
unsigned long last_can_rx_ms = 0;
unsigned long last_speed_send_ms = 0;

// 中央制御基板や各機構の CAN ID
// can_ring.csv / can_yagura.csv に合わせている
const uint16_t CENTRAL_CONTROL_CAN_ID = 0x000;
const uint16_t RING_HAND_1_CAN_ID = 0x200;
const uint16_t RING_HAND_2_CAN_ID = 0x201;
const uint16_t RING_LIFT_1_CAN_ID = 0x300;
const uint16_t RING_LIFT_2_CAN_ID = 0x301;
const uint16_t YAGURA_HAND_1_CAN_ID = 0x400;
const uint16_t YAGURA_HAND_2_CAN_ID = 0x401;
const uint16_t YAGURA_LIFT_CAN_ID = 0x500;

// DM モータ 4 台の ID
// doc.md の 0,1,2,3 の順番に対応
// 0 = 前左, 1 = 前右, 2 = 後右, 3 = 後左
const int MOTOR_COUNT = 4;
const uint16_t OMNI_MOTOR_CAN_IDS[MOTOR_COUNT] = {0x010, 0x011, 0x012, 0x013};
const uint16_t OMNI_FEEDBACK_IDS[MOTOR_COUNT] = {0x010, 0x011, 0x012, 0x013};

// DM モータ制御で使う ID / レジスタ
// speed mode の設定と速度指令送信に使う
const uint16_t DM_CONFIG_WRITE_ID = 0x7FF;
const uint16_t DM_SPEED_COMMAND_BASE_ID = 0x200;
const uint32_t DM_CTRL_MODE_REGISTER = 0x0A;
const uint32_t DM_CTRL_MODE_VELOCITY = 3;

// doc.md のオムニパラメータ
// 機体寸法が変わったらここも見直す
const float OMNI_WHEEL_RADIUS_M = 0.051f;
const float OMNI_CENTER_TO_WHEEL_M = 0.2325f;

// スティック最大時の機体目標速度
// 速くしたい/遅くしたいときはまずここを調整する
const float MAX_LINEAR_SPEED_M_S = 0.6f;
const float MAX_ANGULAR_SPEED_RAD_S = 1.2f;

// 4輪合成後の車輪速度上限
// 一部の車輪だけ危険に速くならないようにするための上限
const float MAX_WHEEL_SPEED_RAD_S = 30.0f;

// スティックの遊び
// 微小な入力ノイズで勝手に動かないようにする
const float STICK_DEADZONE = 0.12f;

// 現在の4輪目標速度 [rad/s]
// updateOmniTargetsFromController() が更新し、
// sendAllWheelSpeeds() がこの値を送る
float wheel_target_rad_s[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

// =========================================================
// 2. 低レベルの共通関数
// =========================================================

void onPs5Connect()
{
  // 接続確認用のメッセージ
  Serial.println("PS5 connected");
}

void onPs5Disconnect()
{
  // 切断確認用のメッセージ
  Serial.println("PS5 disconnected");
}

float applyDeadzone(int8_t raw_value)
{
  // PS5 のスティック値を -1.0 ～ 1.0 に正規化して、
  // 小さい入力は 0 扱いにする
  // 返り値は「計算用に使いやすいスティックの強さ」
  float normalized = static_cast<float>(raw_value) / 127.0f;
  float abs_value = std::fabs(normalized);

  if (abs_value < STICK_DEADZONE)
  {
    return 0.0f;
  }

  float sign = 1.0f;
  if (normalized < 0.0f)
  {
    sign = -1.0f;
  }

  float scaled = (abs_value - STICK_DEADZONE) / (1.0f - STICK_DEADZONE);
  return sign * scaled;
}

void clearWheelTargets()
{
  // 安全のため 4 輪目標速度を全部 0 にする
  // PS5 が切れたときなどに使う
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    wheel_target_rad_s[i] = 0.0f;
  }
}

bool isOmniFeedbackId(uint32_t can_id)
{
  // 受信 CAN ID が足回り 4 台のどれかかどうかを調べる
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    if (OMNI_FEEDBACK_IDS[i] == can_id)
    {
      return true;
    }
  }
  return false;
}

bool sendCan8Bytes(uint16_t can_id,
                   uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
                   uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
  // 機構系の CAN コマンドはほぼ全部 8 バイト固定長で送る
  // can_id が送信先、d0～d7 がデータ本体
  twai_message_t message = {};
  message.identifier = can_id;
  message.extd = 0;
  message.rtr = 0;
  message.data_length_code = 8;
  message.data[0] = d0;
  message.data[1] = d1;
  message.data[2] = d2;
  message.data[3] = d3;
  message.data[4] = d4;
  message.data[5] = d5;
  message.data[6] = d6;
  message.data[7] = d7;

  bool ok = twai_transmit(&message, 0) == ESP_OK;
  if (ok)
  {
    Serial.printf("CAN Sent: ID 0x%03lX Data: %u, %u, %u, %u, %u, %u, %u, %u\n",
                  static_cast<unsigned long>(can_id),
                  d0, d1, d2, d3, d4, d5, d6, d7);
  }
  return ok;
}

bool sendCan4Bytes(uint16_t can_id, const uint8_t data[4])
{
  // DM モータの速度モードは float 4 バイトを送る
  // 速度指令専用の小さい送信関数
  twai_message_t message = {};
  message.identifier = can_id;
  message.extd = 0;
  message.rtr = 0;
  message.data_length_code = 4;
  std::memcpy(message.data, data, 4);
  return twai_transmit(&message, 0) == ESP_OK;
}

bool sendDmSpecialCommand(uint16_t motor_can_id, uint8_t command)
{
  // 0xFC = Enable, 0xFD = Disable, 0xFE = Zero 系
  // setup() では 0xFC を送ってモータを有効化する
  bool ok = sendCan8Bytes(motor_can_id, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command);
  Serial.printf("[DM-S3519] special cmd=0x%02X id=0x%03X (%s)\n",
                command,
                motor_can_id,
                ok ? "OK" : "FAIL");
  return ok;
}

void writeDmVelocityMode(uint16_t motor_can_id)
{
  // DM モータを速度モードに設定する
  // setup() のとき毎回送っている
  bool ok = sendCan8Bytes(
      DM_CONFIG_WRITE_ID,
      static_cast<uint8_t>(motor_can_id & 0xFF),
      static_cast<uint8_t>((motor_can_id >> 8) & 0xFF),
      0x55,
      static_cast<uint8_t>(DM_CTRL_MODE_REGISTER),
      static_cast<uint8_t>(DM_CTRL_MODE_VELOCITY & 0xFF),
      static_cast<uint8_t>((DM_CTRL_MODE_VELOCITY >> 8) & 0xFF),
      static_cast<uint8_t>((DM_CTRL_MODE_VELOCITY >> 16) & 0xFF),
      static_cast<uint8_t>((DM_CTRL_MODE_VELOCITY >> 24) & 0xFF));

  Serial.printf("[DM-S3519] write CTRL_MODE=3 id=0x%03X (%s)\n",
                motor_can_id,
                ok ? "OK" : "FAIL");
}

void sendDmVelocity(uint16_t motor_can_id, float target_rad_s)
{
  // speed mode の本体データは float 4 バイト
  // target_rad_s は車輪角速度 [rad/s]
  uint8_t payload[4] = {0, 0, 0, 0};
  std::memcpy(payload, &target_rad_s, sizeof(float));

  uint16_t can_id = DM_SPEED_COMMAND_BASE_ID + motor_can_id;
  sendCan4Bytes(can_id, payload);
}

// =========================================================
// 3. PS5 ボタン入力処理
// =========================================================

void handleButtonInput()
{
  // Circle / Square は押した瞬間と離した瞬間で別動作なので、
  // 前回状態を記録してエッジ検出している
  static bool prev_circle = false;
  static bool prev_square = false;

  if (!ps5.isConnected())
  {
    prev_circle = false;
    prev_square = false;
    return;
  }

  bool circle = ps5.Circle();
  bool square = ps5.Square();

  // ここは「ボタンを押したらどの CAN を送るか」の一覧
  // 割り当てを変えたいときは、まずこの関数を見る
  //
  // 現在の割り当て:
  // R1    : リング昇降1 降下
  // L1    : リング昇降2 降下
  // R2    : リングハンド1 閉
  // L2    : リングハンド2 閉
  // Up    : 櫓ハンド1/2 閉
  // Down  : 櫓昇降 降下
  // Circle: リング昇降1 手動開始/停止
  // Square: リング昇降2 手動開始/停止
  if (ps5.event.button_down.r1)
  {
    Serial.println("PS5 input: R1 down");
    sendCan8Bytes(RING_LIFT_1_CAN_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
  }
  else if (ps5.event.button_down.l1)
  {
    Serial.println("PS5 input: L1 down");
    sendCan8Bytes(RING_LIFT_2_CAN_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
  }
  else if (ps5.event.button_down.r2)
  {
    Serial.println("PS5 input: R2 down");
    sendCan8Bytes(RING_HAND_1_CAN_ID, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
  }
  else if (ps5.event.button_down.l2)
  {
    Serial.println("PS5 input: L2 down");
    sendCan8Bytes(RING_HAND_2_CAN_ID, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
  }
  else if (ps5.event.button_down.up)
  {
    Serial.println("PS5 input: Up down");
    sendCan8Bytes(YAGURA_HAND_1_CAN_ID, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
    sendCan8Bytes(YAGURA_HAND_2_CAN_ID, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
  }
  else if (ps5.event.button_down.down)
  {
    Serial.println("PS5 input: Down down");
    sendCan8Bytes(YAGURA_LIFT_CAN_ID, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
  }

  if (circle && !prev_circle)
  {
    Serial.println("PS5 input: Circle down");
    sendCan8Bytes(RING_LIFT_1_CAN_ID, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00);
  }
  else if (!circle && prev_circle)
  {
    Serial.println("PS5 input: Circle up");
    sendCan8Bytes(RING_LIFT_1_CAN_ID, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00);
  }

  if (square && !prev_square)
  {
    Serial.println("PS5 input: Square down");
    sendCan8Bytes(RING_LIFT_2_CAN_ID, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00);
  }
  else if (!square && prev_square)
  {
    Serial.println("PS5 input: Square up");
    sendCan8Bytes(RING_LIFT_2_CAN_ID, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00);
  }

  prev_circle = circle;
  prev_square = square;
}

// =========================================================
// 4. オムニ足回り計算
// =========================================================

void updateOmniTargetsFromController()
{
  // この関数の役割:
  // 「PS5 のスティック入力」→「4 輪の目標速度」
  if (!ps5.isConnected())
  {
    clearWheelTargets();
    return;
  }

  // 左スティックで平行移動、右スティックXで回転
  // vx_m_s    : 左右移動速度
  // vy_m_s    : 前後移動速度
  // omega_rad_s : 機体の回転速度
  float vx_m_s = applyDeadzone(ps5.LStickX()) * MAX_LINEAR_SPEED_M_S;
  float vy_m_s = applyDeadzone(ps5.LStickY()) * MAX_LINEAR_SPEED_M_S;
  float omega_rad_s = applyDeadzone(ps5.RStickX()) * MAX_ANGULAR_SPEED_RAD_S;

  // 中心からホイールまでの距離と角速度から、回転用の速度成分を作る
  float rotation_term = OMNI_CENTER_TO_WHEEL_M * omega_rad_s;

  // 青コートの配置に合わせた 4 輪の接線速度
  // 0 = 前左, 1 = 前右, 2 = 後右, 3 = 後左
  // 回転方向が想定と違うときは、まずこの4式を見る
  // ここがオムニ足回りの中心式
  float wheel_linear_m_s[MOTOR_COUNT];
  wheel_linear_m_s[0] = -vx_m_s - vy_m_s - rotation_term;
  wheel_linear_m_s[1] = -vx_m_s + vy_m_s - rotation_term;
  wheel_linear_m_s[2] = vx_m_s + vy_m_s - rotation_term;
  wheel_linear_m_s[3] = vx_m_s - vy_m_s - rotation_term;

  // DM モータへ送る単位 rad/s に変換する
  // m/s を wheel radius で割って角速度にする
  float max_abs_speed = 0.0f;
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    wheel_target_rad_s[i] = wheel_linear_m_s[i] / OMNI_WHEEL_RADIUS_M;

    float abs_speed = std::fabs(wheel_target_rad_s[i]);
    if (abs_speed > max_abs_speed)
    {
      max_abs_speed = abs_speed;
    }
  }

  // どれか1輪でも上限を超えたら、4輪まとめて縮める
  // こうすると、斜め移動 + 回転で一部の車輪だけ危険に速くなるのを防げる
  // 方向や比率は保ったまま小さくする
  if (max_abs_speed > MAX_WHEEL_SPEED_RAD_S)
  {
    float scale = MAX_WHEEL_SPEED_RAD_S / max_abs_speed;
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
      wheel_target_rad_s[i] = wheel_target_rad_s[i] * scale;
    }
  }
}

void printOmniTargets()
{
  // 今どんな速度を各輪へ送ろうとしているかを表示する
  Serial.printf("[OMNI] w0=%.3f w1=%.3f w2=%.3f w3=%.3f\n",
                wheel_target_rad_s[0],
                wheel_target_rad_s[1],
                wheel_target_rad_s[2],
                wheel_target_rad_s[3]);
}

void sendAllWheelSpeeds()
{
  // 4 輪分の目標速度をそのまま DM モータへ送る
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    sendDmVelocity(OMNI_MOTOR_CAN_IDS[i], wheel_target_rad_s[i]);
  }
}

// =========================================================
// 5. CAN受信処理
// =========================================================

void handleCanRx()
{
  // 受信した 1 本のフレームを拾って、
  // DM フィードバックか、各機構からの完了報告かを見分ける
  twai_message_t message;
  if (twai_receive(&message, 0) != ESP_OK)
  {
    return;
  }

  if (!message.extd && !message.rtr && message.data_length_code >= 1 && isOmniFeedbackId(message.identifier))
  {
    // DM モータからの簡易状態表示
    uint8_t d0 = message.data[0];
    Serial.printf("[DM-S3519] feedback id=0x%03lX d0=0x%02X (id_low=%u err_high=%u)\n",
                  static_cast<unsigned long>(message.identifier),
                  d0,
                  d0 & 0x0F,
                  (d0 >> 4) & 0x0F);
  }

  if (message.identifier == CENTRAL_CONTROL_CAN_ID)
  {
    // 各機構から中央制御基板への「完了報告」
    uint8_t report0 = message.data[0];
    uint8_t report1 = message.data[1];

    if (report0 == 0x00 && report1 == 0x00)
    {
      Serial.println("[CAN REPORT] ring recovery complete");
    }
    else if (report0 == 0x00 && report1 == 0x01)
    {
      Serial.println("[CAN REPORT] ring placement complete");
    }
    else if (report0 == 0x00 && report1 == 0x02)
    {
      Serial.println("[CAN REPORT] ring honmaru placement complete");
    }
    else if (report0 == 0x01 && report1 == 0x00)
    {
      Serial.println("[CAN REPORT] yagura recovery complete");
    }
    else if (report0 == 0x02 && report1 == 0x00)
    {
      Serial.println("[CAN REPORT] yagura placement complete");
    }
  }
}

// =========================================================
// 6. 初期化
// =========================================================

void setupPs5()
{
  // 接続時/切断時のイベントを登録してから begin() する
  ps5.attachOnConnect(onPs5Connect);
  ps5.attachOnDisconnect(onPs5Disconnect);
  ps5.begin(PS5_CONTROLLER_MAC);
}

void setupCan()
{
  // ESP32 の CAN(TWAI) を 1Mbps で開始する
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  {
    twai_start();
    Serial.println("CAN Driver Started!");
  }
}

void enableAllDriveMotors()
{
  // 電源投入直後は少し待ってから設定する
  delay(200);

  // 先に速度モード設定、そのあと Enable
  // setup() の中の「DM モータ初期化」担当
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    writeDmVelocityMode(OMNI_MOTOR_CAN_IDS[i]);
    delay(20);
  }

  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    sendDmSpecialCommand(OMNI_MOTOR_CAN_IDS[i], 0xFC);
    delay(20);
  }
}

// =========================================================
// 7. Arduino の標準関数
// =========================================================

void setup()
{
  // Arduino 起動時に 1 回だけ実行される
  // setup() の並び順が、そのまま起動手順
  Serial.begin(SERIAL_BAUD);

  setupPs5();
  setupCan();
  enableAllDriveMotors();
}

void loop()
{
  // loop() はずっと繰り返し呼ばれる
  // delay() で止めずに、millis() で周期管理している
  unsigned long now = millis();

  if (now - last_controller_read_ms >= CONTROLLER_READ_INTERVAL_MS)
  {
    // PS5 入力の読み取り
    // ボタン処理と足回り目標更新をセットで行う
    last_controller_read_ms = now;
    handleButtonInput();
    updateOmniTargetsFromController();
  }

  if (now - last_can_rx_ms >= RX_INTERVAL_MS)
  {
    // CAN 受信処理
    last_can_rx_ms = now;
    handleCanRx();
  }

  if (now - last_speed_send_ms >= SPEED_SEND_INTERVAL_MS)
  {
    // 4 輪目標速度の表示と送信
    // updateOmniTargetsFromController() が作った値をここで実際に送る
    last_speed_send_ms = now;
    printOmniTargets();
    sendAllWheelSpeeds();
  }
}
