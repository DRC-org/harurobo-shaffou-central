#include <Arduino.h>
#include <ps5Controller.h>
#include <cstring>
#include <cmath>
#include "driver/twai.h"

// 1. ピンの住所（TXは送信、RXは受信）
#define CAN_TX_PIN GPIO_NUM_16
#define CAN_RX_PIN GPIO_NUM_4

const unsigned long CONTROLLER_READ_INTERVAL_MS = 10;
const unsigned long RX_INTERVAL_MS = 2;
const uint32_t SPEED_SEND_INTERVAL_MS = 50;

unsigned long last_controller_read_ms = 0;
unsigned long last_can_rx_ms = 0;
unsigned long last_speed_send_ms = 0;

constexpr uint32_t SERIAL_BAUD = 115200;

constexpr uint32_t CTRL_MODE_REGISTER = 0x0A;
constexpr uint32_t CTRL_MODE_VELOCITY = 3;
constexpr uint16_t CONFIG_WRITE_ID = 0x7FF;
// 速度モード指令フレームIDは 0x200 + CAN_ID
constexpr uint16_t SPEED_CMD_BASE_ID = 0x200;
constexpr uint16_t CENTRAL_CONTROL_CAN_ID = 0x000;
constexpr uint16_t RING_HAND_1_CAN_ID = 0x200;
constexpr uint16_t RING_HAND_2_CAN_ID = 0x201;
constexpr uint16_t RING_LIFT_1_CAN_ID = 0x300;
constexpr uint16_t RING_LIFT_2_CAN_ID = 0x301;
constexpr uint16_t YAGURA_HAND_1_CAN_ID = 0x400;
constexpr uint16_t YAGURA_HAND_2_CAN_ID = 0x401;
constexpr uint16_t YAGURA_LIFT_CAN_ID = 0x500;
constexpr uint16_t OMNI_MOTOR_CAN_IDS[] = {0x010, 0x011, 0x012, 0x013};
constexpr uint16_t OMNI_FEEDBACK_IDS[] = {0x010, 0x011, 0x012, 0x013};
constexpr size_t OMNI_WHEEL_COUNT = sizeof(OMNI_MOTOR_CAN_IDS) / sizeof(OMNI_MOTOR_CAN_IDS[0]);

constexpr float OMNI_WHEEL_RADIUS_M = 0.051f;
constexpr float OMNI_CENTER_TO_WHEEL_M = 0.2325f;
constexpr float MAX_LINEAR_SPEED_M_S = 0.6f;
constexpr float MAX_ANGULAR_SPEED_RAD_S = 1.2f;
constexpr float MAX_WHEEL_SPEED_RAD_S = 30.0f;
constexpr float STICK_DEADZONE = 0.12f;

float g_wheel_speed_targets_rad_s[OMNI_WHEEL_COUNT] = {};

constexpr char PS5_CONTROLLER_MAC[] = "7c:66:ef:84:dc:16";

void onPs5Connect()
{
  Serial.println("PS5 connected");
}

void onPs5Disconnect()
{
  Serial.println("PS5 disconnected");
}

float apply_deadzone(int8_t raw)
{
  const float normalized = static_cast<float>(raw) / 127.0f;
  const float abs_value = std::fabs(normalized);
  if (abs_value < STICK_DEADZONE)
  {
    return 0.0f;
  }

  const float sign = normalized >= 0.0f ? 1.0f : -1.0f;
  const float remapped = (abs_value - STICK_DEADZONE) / (1.0f - STICK_DEADZONE);
  return sign * remapped;
}

// 2. 手紙を送る関数（uint32_tはID用、uint8_tはデータ用）
void sendCanMessage(uint32_t msg_id, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7)
{
  twai_message_t message;
  message.identifier = msg_id;
  message.extd = 0;
  message.data_length_code = 8;
  message.data[0] = data0;
  message.data[1] = data1;
  message.data[2] = data2;
  message.data[3] = data3;
  message.data[4] = data4;
  message.data[5] = data5;
  message.data[6] = data6;
  message.data[7] = data7;

  /*
    箱の準備,           CanMsg msg;,             twai_message_t message;
    IDを入れる,       msg.id = 0x101;,           message.identifier = 0x101;
    長さを入れる,      msg.data_length = 2;,     message.data_length_code = 2;
    データを入れる,    msg.data[0] = 0x30;,      message.data[0] = 0x30;
    送信する,         CAN.write(msg);,           twai_transmit(&message, ...);
  */

  // 住所(&)を教えて、10ミリ秒だけ粘って送信！
  if (twai_transmit(&message, 0) == ESP_OK)
  {
    // %Xは16進数、%dは10進数で表示
    Serial.printf("CAN Sent: ID 0x%X Data: %d, %d, %d, %d, %d, %d, %d, %d\n", msg_id, data0, data1, data2, data3, data4, data5, data6, data7);
  }
}

void handleButtonInput()
{
  static bool prev_circle = false;
  static bool prev_square = false;

  if (!ps5.isConnected())
  {
    prev_circle = false;
    prev_square = false;
    return;
  }

  const bool circle = ps5.Circle();
  const bool square = ps5.Square();

  if (ps5.event.button_down.r1)
  {
    Serial.println("PS5 input: R1 down");
    sendCanMessage(RING_LIFT_1_CAN_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
  }
  else if (ps5.event.button_down.l1)
  {
    Serial.println("PS5 input: L1 down");
    sendCanMessage(RING_LIFT_2_CAN_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
  }
  else if (ps5.event.button_down.r2)
  {
    Serial.println("PS5 input: R2 down");
    sendCanMessage(RING_HAND_1_CAN_ID, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
  }
  else if (ps5.event.button_down.l2)
  {
    Serial.println("PS5 input: L2 down");
    sendCanMessage(RING_HAND_2_CAN_ID, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
  }
  else if (ps5.event.button_down.up)
  {
    Serial.println("PS5 input: Up down");
    sendCanMessage(YAGURA_HAND_1_CAN_ID, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
    sendCanMessage(YAGURA_HAND_2_CAN_ID, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
  }
  else if (ps5.event.button_down.down)
  {
    Serial.println("PS5 input: Down down");
    sendCanMessage(YAGURA_LIFT_CAN_ID, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
  }

  if (circle && !prev_circle)
  {
    Serial.println("PS5 input: Circle down");
    sendCanMessage(RING_LIFT_1_CAN_ID, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00);
  }
  else if (!circle && prev_circle)
  {
    Serial.println("PS5 input: Circle up");
    sendCanMessage(RING_LIFT_1_CAN_ID, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00);
  }

  if (square && !prev_square)
  {
    Serial.println("PS5 input: Square down");
    sendCanMessage(RING_LIFT_2_CAN_ID, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00);
  }
  else if (!square && prev_square)
  {
    Serial.println("PS5 input: Square up");
    sendCanMessage(RING_LIFT_2_CAN_ID, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00);
  }

  prev_circle = circle;
  prev_square = square;
}

void updateOmniTargetsFromController()
{
  if (!ps5.isConnected())
  {
    for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
    {
      g_wheel_speed_targets_rad_s[i] = 0.0f;
    }
    return;
  }

  const float vx_m_s = apply_deadzone(ps5.LStickX()) * MAX_LINEAR_SPEED_M_S;
  const float vy_m_s = apply_deadzone(ps5.LStickY()) * MAX_LINEAR_SPEED_M_S;
  const float omega_rad_s = apply_deadzone(ps5.RStickX()) * MAX_ANGULAR_SPEED_RAD_S;
  const float rotation_term = OMNI_CENTER_TO_WHEEL_M * omega_rad_s;

  // 青コート図の 0=前左, 1=前右, 2=後右, 3=後左 を基準にした X-drive オムニ変換
  const float wheel_linear_m_s[OMNI_WHEEL_COUNT] = {
      -vx_m_s - vy_m_s - rotation_term,
      -vx_m_s + vy_m_s - rotation_term,
      vx_m_s + vy_m_s - rotation_term,
      vx_m_s - vy_m_s - rotation_term,
  };

  float max_abs_wheel_speed_rad_s = 0.0f;
  for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
  {
    g_wheel_speed_targets_rad_s[i] = wheel_linear_m_s[i] / OMNI_WHEEL_RADIUS_M;
    max_abs_wheel_speed_rad_s = std::max(max_abs_wheel_speed_rad_s, std::fabs(g_wheel_speed_targets_rad_s[i]));
  }

  if (max_abs_wheel_speed_rad_s > MAX_WHEEL_SPEED_RAD_S)
  {
    const float scale = MAX_WHEEL_SPEED_RAD_S / max_abs_wheel_speed_rad_s;
    for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
    {
      g_wheel_speed_targets_rad_s[i] *= scale;
    }
  }
}

bool is_omni_feedback_id(uint32_t can_id)
{
  for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
  {
    if (OMNI_FEEDBACK_IDS[i] == can_id)
    {
      return true;
    }
  }
  return false;
}

void poll_feedback(const twai_message_t &rx)
{
  if (rx.extd || rx.rtr || rx.data_length_code < 1 || !is_omni_feedback_id(rx.identifier))
  {
    return;
  }

  const uint8_t d0 = rx.data[0];
  Serial.printf(
      "[DM-S3519] feedback id=0x%03X d0=0x%02X (id_low=%u err_high=%u)\n",
      rx.identifier,
      d0,
      d0 & 0x0F,
      (d0 >> 4) & 0x0F);
}

void handleCanRx()
{
  // 以下は、自動操作が終わったのを各arduinoからうけとり、こちらでも自動操作終わったことにするプログラム
  twai_message_t rx_msg; // 受信用メッセージの箱

  if (twai_receive(&rx_msg, 0) == ESP_OK)
  {                                    // メッセージが届いているか確認（待ち時間0で一瞬だけ確認）
    uint32_t rxId = rx_msg.identifier; // Arduinoの rxId = msg.id と同じ

    poll_feedback(rx_msg);

    if (rxId == CENTRAL_CONTROL_CAN_ID)
    { // 各基板から中央制御基板への完了報告
      const uint8_t report0 = rx_msg.data[0];
      const uint8_t report1 = rx_msg.data[1];

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
}

bool send_can_standard_8bytes(uint16_t id, const uint8_t data[8])
{
  twai_message_t tx{};
  tx.identifier = id;
  tx.extd = 0;
  tx.rtr = 0;
  tx.data_length_code = 8;
  std::memcpy(tx.data, data, 8);
  return twai_transmit(&tx, 0) == ESP_OK;
}

///  「モーターへのリモコン命令」です。
///  この関数はCANで
///  FF FF FF FF FF FF FF コマンド
///  という決まった形の8バイトを送って、モーター状態を切り替えます。
///  - 0xFC で有効化（回せる状態）
///  - 0xFD で無効化
///  - 0xFE でゼロ位置系の指令
///  このコードでは setup() で 0xFC を送って、最初にモーターを有効化しています。
bool send_special_motor_command(uint16_t motor_can_id, uint8_t command)
{
  // DM系で使われる特殊コマンド形式 (0xFC:Enable / 0xFD:Disable / 0xFE:Zero)
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command};
  const bool ok = send_can_standard_8bytes(motor_can_id, data);
  Serial.printf("[DM-S3519] special cmd=0x%02X id=0x%03X (%s)\n", command, motor_can_id, ok ? "OK" : "FAIL");
  return ok;
}

bool send_can_standard_4bytes(uint16_t id, const uint8_t data[4])
{
  twai_message_t tx{};
  tx.identifier = id;
  tx.extd = 0;
  tx.rtr = 0;
  tx.data_length_code = 4;
  std::memcpy(tx.data, data, 4);
  return twai_transmit(&tx, 0) == ESP_OK;
}

void write_ctrl_mode_velocity(uint16_t motor_can_id)
{
  // CAN設定コマンド:
  // ID=0x7FF, D2=0x55(write), D3=RID(0x0A), D4..D7=3(velocity mode)
  uint8_t data[8] = {};
  data[0] = static_cast<uint8_t>(motor_can_id & 0xFF);
  data[1] = static_cast<uint8_t>((motor_can_id >> 8) & 0xFF);
  data[2] = 0x55;
  data[3] = static_cast<uint8_t>(CTRL_MODE_REGISTER);
  data[4] = static_cast<uint8_t>(CTRL_MODE_VELOCITY & 0xFF);
  data[5] = static_cast<uint8_t>((CTRL_MODE_VELOCITY >> 8) & 0xFF);
  data[6] = static_cast<uint8_t>((CTRL_MODE_VELOCITY >> 16) & 0xFF);
  data[7] = static_cast<uint8_t>((CTRL_MODE_VELOCITY >> 24) & 0xFF);

  const bool ok = send_can_standard_8bytes(CONFIG_WRITE_ID, data);
  Serial.printf("[DM-S3519] write CTRL_MODE=3 id=0x%03X (%s)\n", motor_can_id, ok ? "OK" : "FAIL");
}

void send_velocity(uint16_t motor_can_id, float v_des_rad_s)
{
  // 速度モードでは payload は float(4byte, little-endian)
  uint8_t payload[4] = {};
  std::memcpy(payload, &v_des_rad_s, sizeof(float));
  const uint16_t id = SPEED_CMD_BASE_ID + motor_can_id;
  send_can_standard_4bytes(id, payload);
}

void setup()
{
  Serial.begin(SERIAL_BAUD);

  ps5.attachOnConnect(onPs5Connect);
  ps5.attachOnDisconnect(onPs5Disconnect);
  ps5.begin(PS5_CONTROLLER_MAC);

  // 3. 3つの書類（設定）を作成

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // 4. 書類を提出(install)して、OKならスイッチON(start)
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  {
    twai_start();
    Serial.println("CAN Driver Started!");
  }

  // モータ側の起動直後を避けるため少し待ってから速度モード設定
  delay(200);
  for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
  {
    write_ctrl_mode_velocity(OMNI_MOTOR_CAN_IDS[i]);
    delay(20);
  }
  for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
  {
    send_special_motor_command(OMNI_MOTOR_CAN_IDS[i], 0xFC);
    delay(20);
  }
}

void loop()
{
  unsigned long now = millis();

  if (now - last_controller_read_ms >= CONTROLLER_READ_INTERVAL_MS)
  {
    last_controller_read_ms = now;
    handleButtonInput();
    updateOmniTargetsFromController();
  }

  if (now - last_can_rx_ms >= RX_INTERVAL_MS)
  {
    last_can_rx_ms = now;
    handleCanRx();
  }

  if (now - last_speed_send_ms >= SPEED_SEND_INTERVAL_MS)
  {
    last_speed_send_ms = now;
    Serial.printf(
        "[OMNI] w0=%.3f w1=%.3f w2=%.3f w3=%.3f\n",
        g_wheel_speed_targets_rad_s[0],
        g_wheel_speed_targets_rad_s[1],
        g_wheel_speed_targets_rad_s[2],
        g_wheel_speed_targets_rad_s[3]);
    for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
    {
      send_velocity(OMNI_MOTOR_CAN_IDS[i], g_wheel_speed_targets_rad_s[i]);
    }
  }
}
