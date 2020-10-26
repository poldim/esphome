#include "sht20.h"
#include "esphome/core/log.h"

namespace esphome {
namespace sht20 {

static const char *TAG = "sht20";

static const uint16_t sht20_COMMAND_READ_SERIAL_NUMBER = 0x3780;
static const uint16_t sht20_COMMAND_READ_STATUS = 0xF32D;
static const uint16_t sht20_COMMAND_CLEAR_STATUS = 0x3041;
static const uint16_t sht20_COMMAND_HEATER_ENABLE = 0x306D;
static const uint16_t sht20_COMMAND_HEATER_DISABLE = 0x3066;
static const uint16_t sht20_COMMAND_SOFT_RESET = 0x30A2;
static const uint16_t sht20_COMMAND_POLLING_H = 0x2400;
static const uint16_t sht20_COMMAND_FETCH_DATA = 0xE000;

void sht20Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up sht20...");
  if (!this->write_command_(sht20_COMMAND_READ_SERIAL_NUMBER)) {
    this->mark_failed();
    return;
  }

  uint16_t raw_serial_number[2];
  if (!this->read_data_(raw_serial_number, 2)) {
    this->mark_failed();
    return;
  }
  uint32_t serial_number = (uint32_t(raw_serial_number[0]) << 16) | uint32_t(raw_serial_number[1]);
  ESP_LOGV(TAG, "    Serial Number: 0x%08X", serial_number);
}
void sht20Component::dump_config() {
  ESP_LOGCONFIG(TAG, "sht20:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with sht20 failed!");
  }
  LOG_UPDATE_INTERVAL(this);

  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
}
float sht20Component::get_setup_priority() const { return setup_priority::DATA; }
void sht20Component::update() {
  if (this->status_has_warning()) {
    ESP_LOGD(TAG, "Retrying to reconnect the sensor.");
    this->write_command_(sht20_COMMAND_SOFT_RESET);
  }
  if (!this->write_command_(sht20_COMMAND_POLLING_H)) {
    this->status_set_warning();
    return;
  }

  this->set_timeout(50, [this]() {
    uint16_t raw_data[2];
    if (!this->read_data_(raw_data, 2)) {
      this->status_set_warning();
      return;
    }

    float temperature = 175.72f * float(raw_data[0]) / 65536.0f - 46.85f;
    float humidity = 125.0f * float(raw_data[1]) / 65536.0f - 6.0f;

    ESP_LOGD(TAG, "Got temperature=%.2f°C humidity=%.2f%%", temperature, humidity);
    if (this->temperature_sensor_ != nullptr)
      this->temperature_sensor_->publish_state(temperature);
    if (this->humidity_sensor_ != nullptr)
      this->humidity_sensor_->publish_state(humidity);
    this->status_clear_warning();
  });
}

bool sht20Component::write_command_(uint16_t command) {
  // Warning ugly, trick the I2Ccomponent base by setting register to the first 8 bit.
  return this->write_byte(command >> 8, command & 0xFF);
}

uint8_t sht_crc(uint8_t data1, uint8_t data2) {
  uint8_t bit;
  uint8_t crc = 0xFF;

  crc ^= data1;
  for (bit = 8; bit > 0; --bit) {
    if (crc & 0x80)
      crc = (crc << 1) ^ 0x131;
    else
      crc = (crc << 1);
  }

  crc ^= data2;
  for (bit = 8; bit > 0; --bit) {
    if (crc & 0x80)
      crc = (crc << 1) ^ 0x131;
    else
      crc = (crc << 1);
  }

  return crc;
}

bool sht20Component::read_data_(uint16_t *data, uint8_t len) {
  const uint8_t num_bytes = len * 3;
  auto *buf = new uint8_t[num_bytes];

  if (!this->parent_->raw_receive(this->address_, buf, num_bytes)) {
    delete[](buf);
    return false;
  }

  for (uint8_t i = 0; i < len; i++) {
    const uint8_t j = 3 * i;
    uint8_t crc = sht_crc(buf[j], buf[j + 1]);
    if (crc != buf[j + 2]) {
      ESP_LOGE(TAG, "CRC8 Checksum invalid! 0x%02X != 0x%02X", buf[j + 2], crc);
      delete[](buf);
      return false;
    }
    data[i] = (buf[j] << 8) | buf[j + 1];
  }

  delete[](buf);
  return true;
}

}  // namespace sht20
}  // namespace esphome
