#include <Arduino.h>
#include <cstring>
#include "driver/twai.h"

#define CAN_TX_PIN GPIO_NUM_16
#define CAN_RX_PIN GPIO_NUM_4

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint16_t CONFIG_WRITE_ID = 0x7FF;
constexpr uint16_t READ_COMMAND = 0x33;
constexpr uint32_t MST_ID_REGISTER = 0x07;
constexpr uint32_t ESC_ID_REGISTER = 0x08;
constexpr uint32_t CTRL_MODE_REGISTER = 0x0A;
constexpr uint16_t CURRENT_CAN_ID = 0x001;
constexpr uint16_t NEW_CAN_ID = 0x013;
constexpr uint16_t NEW_MST_ID = 0x013;

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

void log_can_rx(const twai_message_t &rx)
{
  Serial.printf(
      "[CAN RX] id=0x%03lX dlc=%u extd=%u rtr=%u data=",
      static_cast<unsigned long>(rx.identifier),
      rx.data_length_code,
      rx.extd,
      rx.rtr);
  for (uint8_t i = 0; i < rx.data_length_code; ++i)
  {
    Serial.printf("%02X", rx.data[i]);
    if (i + 1 < rx.data_length_code)
    {
      Serial.print(" ");
    }
  }
  Serial.println();
}

bool write_motor_register_u32(uint16_t motor_can_id, uint8_t reg, uint32_t value)
{
  uint8_t data[8] = {};
  data[0] = static_cast<uint8_t>(motor_can_id & 0xFF);
  data[1] = static_cast<uint8_t>((motor_can_id >> 8) & 0xFF);
  data[2] = 0x55;
  data[3] = reg;
  data[4] = static_cast<uint8_t>(value & 0xFF);
  data[5] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[6] = static_cast<uint8_t>((value >> 16) & 0xFF);
  data[7] = static_cast<uint8_t>((value >> 24) & 0xFF);

  const bool ok = send_can_standard_8bytes(CONFIG_WRITE_ID, data);
  Serial.printf(
      "[DM-S3519] write reg=0x%02X target=0x%03X value=0x%08lX (%s)\n",
      reg,
      motor_can_id,
      static_cast<unsigned long>(value),
      ok ? "OK" : "FAIL");
  return ok;
}

void send_special_motor_command(uint16_t motor_can_id, uint8_t command)
{
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command};
  const bool ok = send_can_standard_8bytes(motor_can_id, data);
  Serial.printf("[DM-S3519] special cmd=0x%02X id=0x%03X (%s)\n", command, motor_can_id, ok ? "OK" : "FAIL");
}

void save_motor_parameters(uint16_t motor_can_id)
{
  uint8_t data[8] = {};
  data[0] = static_cast<uint8_t>(motor_can_id & 0xFF);
  data[1] = static_cast<uint8_t>((motor_can_id >> 8) & 0xFF);
  data[2] = 0xAA;
  data[3] = 0x01;

  const bool ok = send_can_standard_8bytes(CONFIG_WRITE_ID, data);
  Serial.printf("[DM-S3519] save params target=0x%03X (%s)\n", motor_can_id, ok ? "OK" : "FAIL");
}

bool read_motor_register_u32(uint16_t motor_can_id, uint8_t reg)
{
  uint8_t data[8] = {};
  data[0] = static_cast<uint8_t>(motor_can_id & 0xFF);
  data[1] = static_cast<uint8_t>((motor_can_id >> 8) & 0xFF);
  data[2] = READ_COMMAND;
  data[3] = reg;

  const bool ok = send_can_standard_8bytes(CONFIG_WRITE_ID, data);
  Serial.printf("[DM-S3519] read reg=0x%02X target=0x%03X (%s)\n", reg, motor_can_id, ok ? "OK" : "FAIL");
  return ok;
}

void drain_can_rx(unsigned long duration_ms)
{
  const unsigned long start = millis();
  while (millis() - start < duration_ms)
  {
    twai_message_t rx{};
    if (twai_receive(&rx, pdMS_TO_TICKS(10)) == ESP_OK)
    {
      log_can_rx(rx);
    }
  }
}

void readback_sequence(uint16_t motor_can_id, const char *label)
{
  Serial.printf("[DM-S3519] readback begin target=0x%03X label=%s\n", motor_can_id, label);
  read_motor_register_u32(motor_can_id, ESC_ID_REGISTER);
  delay(20);
  read_motor_register_u32(motor_can_id, MST_ID_REGISTER);
  delay(20);
  read_motor_register_u32(motor_can_id, CTRL_MODE_REGISTER);
  drain_can_rx(300);
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
  send_special_motor_command(CURRENT_CAN_ID, 0xFD);
  delay(50);
  write_motor_register_u32(CURRENT_CAN_ID, ESC_ID_REGISTER, NEW_CAN_ID);
  delay(50);
  write_motor_register_u32(NEW_CAN_ID, MST_ID_REGISTER, NEW_MST_ID);
  delay(50);
  save_motor_parameters(NEW_CAN_ID);
  delay(50);
  drain_can_rx(300);

  readback_sequence(CURRENT_CAN_ID, "old-id");
  readback_sequence(NEW_CAN_ID, "new-id");

  Serial.println("[DM-S3519] ID write sequence finished.");
}

void loop()
{
}
