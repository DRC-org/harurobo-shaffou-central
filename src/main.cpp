#include <Arduino.h>
#include <PS4Controller.h>
#include <cstring>
#include <esp_mac.h>
#include "driver/twai.h"

bool at_run = false; // 〇ボタンやその他を押すと、自動の一連の動作が起こる。□ボタンでは手動操作になる。自動操作中に□ボタンの影響が出るのを防ぐ。

// 1. ピンの住所（TXは送信、RXは受信）
#define CAN_TX_PIN GPIO_NUM_16
#define CAN_RX_PIN GPIO_NUM_4

const unsigned long CONTROLLER_READ_INTERVAL_MS = 10;
const unsigned long RX_INTERVAL_MS = 2;
const uint32_t SPEED_SEND_INTERVAL_MS = 50;

unsigned long last_controller_read_ms = 0;
unsigned long last_can_rx_ms = 0;
unsigned long last_speed_send_ms = 0;

// CANトランシーバ接続ピン
constexpr uint16_t MOTOR_CAN_ID = 0x001;
constexpr uint32_t SERIAL_BAUD = 115200;

constexpr uint32_t CTRL_MODE_REGISTER = 0x0A;
constexpr uint32_t CTRL_MODE_VELOCITY = 3;
constexpr uint16_t CONFIG_WRITE_ID = 0x7FF;
// 速度モード指令フレームIDは 0x200 + CAN_ID
constexpr uint16_t SPEED_CMD_BASE_ID = 0x200;
// マニュアル既定のフィードバックID (MST_ID)
constexpr uint16_t FEEDBACK_ID_DEFAULT = 0x000;

struct FeedbackState
{
  bool valid = false;
  uint16_t can_id = FEEDBACK_ID_DEFAULT;
  uint8_t d0 = 0;
  uint8_t id_low = 0;
  uint8_t err_high = 0;
  unsigned long updated_ms = 0;
};

FeedbackState g_feedback_state;

void onPs4Connect()
{
  Serial.println("PS4 connected");
}

void onPs4Disconnect()
{
  Serial.println("PS4 disconnected");
}

void printBluetoothMac()
{
  uint8_t bt_mac[6] = {};
  if (esp_read_mac(bt_mac, ESP_MAC_BT) != ESP_OK)
  {
    Serial.println("Failed to read ESP32 BT MAC");
    return;
  }

  Serial.printf(
      "ESP32 BT MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
      bt_mac[0],
      bt_mac[1],
      bt_mac[2],
      bt_mac[3],
      bt_mac[4],
      bt_mac[5]);
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
  if (twai_transmit(&message, pdMS_TO_TICKS(10)) == ESP_OK)
  {
    // %Xは16進数、%dは10進数で表示
    Serial.printf("CAN Sent: ID 0x%X Data: %d, %d, %d, %d, %d, %d, %d, %d\n", msg_id, data0, data1, data2, data3, data4, data5, data6, data7);
  }
}

/// 未使用
void handleAutoInput()
{
  if (!PS4.isConnected())
  {
    return;
  }

  if (PS4.event.button_down.r1)
  { // r1ボタンで機体右側のリングを回収する 0x200 0x300
    at_run = true;
    sendCanMessage(0x300, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
  }
  else if (PS4.event.button_down.l1)
  { // l1ボタンで機体左側のリングを回収する 0x201 0x301
    sendCanMessage(0x301, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
    at_run = true;
  }
  else if (PS4.event.button_down.r2)
  { // r2ボタンで機体右側のリングを設置する(〇ボタンの操作である程度櫓に近づけた後にやる操作なのでいきなりリングハンドSVMDにCANを送っている)
    at_run = true;
    sendCanMessage(0x200, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
  }
  else if (PS4.event.button_down.l2)
  { // l2ボタンで機体左側のリングを設置する(□ボタンの操作である程度櫓に近づけた後にやる操作なのでいきなりリングハンドSVMDにCANを送っている)
    at_run = true;
    sendCanMessage(0x201, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
  }
  else if (PS4.event.button_down.up)
  { // 十字キー上で櫓を回収する
    at_run = true;
    sendCanMessage(0x400, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
  }
  else if (PS4.event.button_down.down)
  { // 十字キー下で櫓を設置する
    at_run = true;
    sendCanMessage(0x500, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
  }
}

void handleControllerInput()
{
  if (!PS4.isConnected() || at_run == true)
  {
    return;
  }

  if (PS4.event.button_down.circle)
  {
    // 手動操作
    // モーター動く
    sendCanMessage(0x300, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00); // 右リング昇降を手動操作
  }
  else if (PS4.event.button_up.circle)
  {
    // モーター停止
    sendCanMessage(0x300, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00); // 右リング昇降を手動操作の取りやめ
  }

  if (PS4.event.button_down.square)
  {
    // 手動操作
    // モーター動く
    sendCanMessage(0x301, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00); // 左リング昇降を手動操作
  }
  else if (PS4.event.button_up.square)
  {
    // モーター停止
    sendCanMessage(0x301, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00); // 左リング昇降を手動操作の取りやめ
  }
}

void poll_feedback(const twai_message_t &rx)
{
  if (rx.extd || rx.rtr)
  {
    return;
  }
  if (rx.identifier != FEEDBACK_ID_DEFAULT || rx.data_length_code < 1)
  {
    return;
  }

  const uint8_t d0 = rx.data[0];
  g_feedback_state.valid = true;
  g_feedback_state.can_id = rx.identifier;
  g_feedback_state.d0 = d0;
  g_feedback_state.id_low = d0 & 0x0F;
  g_feedback_state.err_high = (d0 >> 4) & 0x0F;
  g_feedback_state.updated_ms = millis();

  Serial.printf(
      "[DM-S3519] feedback id=0x%03X d0=0x%02X (id_low=%u err_high=%u)\n",
      g_feedback_state.can_id,
      g_feedback_state.d0,
      g_feedback_state.id_low,
      g_feedback_state.err_high);
}

void handleCanRx()
{
  // 以下は、自動操作が終わったのを各arduinoからうけとり、こちらでも自動操作終わったことにするプログラム
  twai_message_t rx_msg; // 受信用メッセージの箱

  if (twai_receive(&rx_msg, 0) == ESP_OK)
  {                                    // メッセージが届いているか確認（待ち時間0で一瞬だけ確認）
    uint32_t rxId = rx_msg.identifier; // Arduinoの rxId = msg.id と同じ

    poll_feedback(rx_msg);

    if (rxId == 0x000)
    { // 0x000は中央制御基板のCAN_ID
      uint8_t command_0 = rx_msg.data[0];
      uint8_t command_1 = rx_msg.data[1];
      uint8_t command_2 = rx_msg.data[2];
      if (command_0 == 0x00 && command_1 == 0x00 && command_2 == 0x00)
      { // 右リング回収操作終了
        at_run = false;
      }
      else if (command_0 == 0x00 && command_1 == 0x01 && command_2 == 0x00)
      { // 右リング設置操作終了
        at_run = false;
      }
      else if (command_0 == 0x00 && command_1 == 0x00 && command_2 == 0x01)
      { // 左リング回収操作終了
        at_run = false;
      }
      else if (command_0 == 0x00 && command_1 == 0x01 && command_2 == 0x01)
      { // 左リング設置操作終了
        at_run = false;
      }
      else if (command_0 == 0x01 && command_1 == 0x00 && command_2 == 0x00)
      { // 十字キー上操作終了
        at_run = false;
      }
      else if (command_0 == 0x02 && command_1 == 0x00 && command_2 == 0x00)
      { // 十字キー下操作終了
        at_run = false;
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
  return twai_transmit(&tx, pdMS_TO_TICKS(10)) == ESP_OK;
}

///  「モーターへのリモコン命令」です。
///  この関数はCANで
///  FF FF FF FF FF FF FF コマンド
///  という決まった形の8バイトを送って、モーター状態を切り替えます。
///  - 0xFC で有効化（回せる状態）
///  - 0xFD で無効化
///  - 0xFE でゼロ位置系の指令
///  このコードでは setup() で 0xFC を送って、最初にモーターを有効化しています。
bool send_special_motor_command(uint8_t command)
{
  // DM系で使われる特殊コマンド形式 (0xFC:Enable / 0xFD:Disable / 0xFE:Zero)
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command};
  const bool ok = send_can_standard_8bytes(MOTOR_CAN_ID, data);
  Serial.printf("[DM-S3519] special cmd=0x%02X id=0x%03X (%s)\n", command, MOTOR_CAN_ID, ok ? "OK" : "FAIL");
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
  return twai_transmit(&tx, pdMS_TO_TICKS(10)) == ESP_OK;
}

void write_ctrl_mode_velocity()
{
  // CAN設定コマンド:
  // ID=0x7FF, D2=0x55(write), D3=RID(0x0A), D4..D7=3(velocity mode)
  uint8_t data[8] = {};
  data[0] = static_cast<uint8_t>(MOTOR_CAN_ID & 0xFF);
  data[1] = static_cast<uint8_t>((MOTOR_CAN_ID >> 8) & 0xFF);
  data[2] = 0x55;
  data[3] = static_cast<uint8_t>(CTRL_MODE_REGISTER);
  data[4] = static_cast<uint8_t>(CTRL_MODE_VELOCITY & 0xFF);
  data[5] = static_cast<uint8_t>((CTRL_MODE_VELOCITY >> 8) & 0xFF);
  data[6] = static_cast<uint8_t>((CTRL_MODE_VELOCITY >> 16) & 0xFF);
  data[7] = static_cast<uint8_t>((CTRL_MODE_VELOCITY >> 24) & 0xFF);

  const bool ok = send_can_standard_8bytes(CONFIG_WRITE_ID, data);
  Serial.printf("[DM-S3519] write CTRL_MODE=3 (%s)\n", ok ? "OK" : "FAIL");
}

void send_velocity(float v_des_rad_s)
{
  // 速度モードでは payload は float(4byte, little-endian)
  uint8_t payload[4] = {};
  std::memcpy(payload, &v_des_rad_s, sizeof(float));
  const uint16_t id = SPEED_CMD_BASE_ID + MOTOR_CAN_ID;
  const bool ok = send_can_standard_4bytes(id, payload);
  Serial.printf("[DM-S3519] speed=%.3f rad/s id=0x%03X (%s)\n", v_des_rad_s, id, ok ? "OK" : "FAIL");
}

void setup()
{
  Serial.begin(115200);

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

  // 5. 自分のMACアドレスを宣言してBluetooth開始
  PS4.attachOnConnect(onPs4Connect);
  PS4.attachOnDisconnect(onPs4Disconnect);
  printBluetoothMac();
  PS4.begin();

  // モータ側の起動直後を避けるため少し待ってからモード設定
  delay(200);
  write_ctrl_mode_velocity();
  delay(50);
  send_special_motor_command(0xFC); // モーター有効化
}

constexpr float SPEED = 6.283f; // rad/s

void loop()
{
  unsigned long now = millis();

  if (now - last_controller_read_ms >= CONTROLLER_READ_INTERVAL_MS)
  {
    last_controller_read_ms = now;
    handleControllerInput();
  }

  if (now - last_can_rx_ms >= RX_INTERVAL_MS)
  {
    last_can_rx_ms = now;
    handleCanRx();
  }

  if (now - last_speed_send_ms >= SPEED_SEND_INTERVAL_MS)
  {
    last_speed_send_ms = now;
    send_velocity(SPEED);
  }
}

/*
if (rxId == 0x000) {
  uint8_t fromWhom = rx_msg.data[1]; // 誰からの報告か確認
  if (fromWhom == 0) {
     // リング1の自動モード終了
  } else if (fromWhom == 1) {
     // リング2の自動モード終了
  }
}
*/
