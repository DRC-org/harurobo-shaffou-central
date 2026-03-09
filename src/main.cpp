#include <Arduino.h>
#include <PS4Controller.h>
#include "driver/twai.h"

bool at_run = false; // 〇ボタンやその他を押すと、自動の一連の動作が起こる。□ボタンでは手動操作になる。自動操作中に□ボタンの影響が出るのを防ぐ。

// 1. ピンの住所（TXは送信、RXは受信）
#define CAN_TX_PIN GPIO_NUM_16
#define CAN_RX_PIN GPIO_NUM_4

const unsigned long AUTO_INTERVAL_MS = 10;
const unsigned long MANUAL_INTERVAL_MS = 10;
const unsigned long RX_INTERVAL_MS = 2;

unsigned long last_auto_ms = 0;
unsigned long last_manual_ms = 0;
unsigned long last_rx_ms = 0;

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

void handleManualInput()
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

void handleCanRx()
{
  // 以下は、自動操作が終わったのを各arduinoからうけとり、こちらでも自動操作終わったことにするプログラム
  twai_message_t rx_msg; // 受信用メッセージの箱

  if (twai_receive(&rx_msg, 0) == ESP_OK)
  {                                    // メッセージが届いているか確認（待ち時間0で一瞬だけ確認）
    uint32_t rxId = rx_msg.identifier; // Arduinoの rxId = msg.id と同じ

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
  PS4.begin();
}

void loop()
{
  unsigned long now = millis();

  if (now - last_auto_ms >= AUTO_INTERVAL_MS)
  {
    last_auto_ms = now;
    handleAutoInput();
  }

  if (now - last_manual_ms >= MANUAL_INTERVAL_MS)
  {
    last_manual_ms = now;
    handleManualInput();
  }

  if (now - last_rx_ms >= RX_INTERVAL_MS)
  {
    last_rx_ms = now;
    handleCanRx();
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
