#include "afro_esc.h"

#include <ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdint.h>

#include <algorithm.hpp>  // used for min max functions

AfroESC::AfroESC(const int _i2c_handle, const int _i2c_address,
                 const int _pole_pairs) {
  i2c_handle_ = _i2c_handle;
  i2c_address_ = _i2c_address;
  pole_pairs_ = _pole_pairs;
}

bool AfroESC::SetMotorSpeed(double _speed) {
  int result;
  _speed = std::min(std::max(_speed, -1.0), 1.0);
  uint16_t speed_data = 0;
  if (_speed < 0) {
    _speed *= -1;
    speed_data += kInvertDirection;
  }
  speed_data += (int)(kMaxSpeed * _speed);
  uint8_t data[] = {kRegSetSpeed, (uint8_t)(0xFF & (speed_data >> 8)),
                    (uint8_t)(speed_data & 0xFF)};
  struct i2c_msg msg = {i2c_address_, 0, sizeof(data), data};
  struct i2c_rdwr_ioctl_data ioctl_data = {&msg, 1};
  if (ioctl(i2c_handle_, I2C_RDWR, &ioctl_data) != 1) {
    return false;
  }
  return true;
}

int AfroESC::GetMotorComCounter() { return 0; }
double GetMotorRevCounter() { return 0; }
int GetBatteryAdc() { return 0; }
double GetBatteryVoltage() { return 0; }
int GetTemperatureAdc() { return 0; }
int GetID() { return 0; }
bool VerifyID() { return false; }
