#include <Arduino.h>
#include <cstring>
#include "driver/twai.h"

#define CAN_TX_PIN GPIO_NUM_16
#define CAN_RX_PIN GPIO_NUM_4

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint16_t MOTOR_CAN_ID = 0x013;
constexpr uint16_t FEEDBACK_ID = 0x013;
constexpr uint16_t CONFIG_WRITE_ID = 0x7FF;
constexpr uint16_t SPEED_CMD_BASE_ID = 0x200;
constexpr uint32_t CTRL_MODE_REGISTER = 0x0A;
constexpr uint32_t CTRL_MODE_VELOCITY = 3;
constexpr uint32_t RX_INTERVAL_MS = 2;
constexpr uint32_t SPEED_SEND_INTERVAL_MS = 50;
constexpr float TEST_SPEED_RAD_S = 6.283f;

unsigned long last_can_rx_ms = 0;
unsigned long last_speed_send_ms = 0;

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

void write_ctrl_mode_velocity()
{
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

void send_special_motor_command(uint8_t command)
{
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command};
  const bool ok = send_can_standard_8bytes(MOTOR_CAN_ID, data);
  Serial.printf("[DM-S3519] special cmd=0x%02X id=0x%03X (%s)\n", command, MOTOR_CAN_ID, ok ? "OK" : "FAIL");
}

void send_velocity(float v_des_rad_s)
{
  uint8_t payload[4] = {};
  std::memcpy(payload, &v_des_rad_s, sizeof(float));
  const uint16_t id = SPEED_CMD_BASE_ID + MOTOR_CAN_ID;
  const bool ok = send_can_standard_4bytes(id, payload);
  Serial.printf("[DM-S3519] speed=%.3f rad/s id=0x%03X (%s)\n", v_des_rad_s, id, ok ? "OK" : "FAIL");
}

void poll_feedback()
{
  twai_message_t rx{};
  if (twai_receive(&rx, 0) != ESP_OK)
  {
    return;
  }
  if (rx.extd || rx.rtr || rx.identifier != FEEDBACK_ID || rx.data_length_code < 1)
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

void setup()
{
  Serial.begin(SERIAL_BAUD);

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  {
    twai_start();
    Serial.println("CAN Driver Started!");
  }

  delay(200);
  write_ctrl_mode_velocity();
  delay(50);
  send_special_motor_command(0xFC);
}

void loop()
{
  const unsigned long now = millis();

  if (now - last_can_rx_ms >= RX_INTERVAL_MS)
  {
    last_can_rx_ms = now;
    poll_feedback();
  }

  if (now - last_speed_send_ms >= SPEED_SEND_INTERVAL_MS)
  {
    last_speed_send_ms = now;
    send_velocity(TEST_SPEED_RAD_S);
  }
}
