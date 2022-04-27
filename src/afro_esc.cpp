#include "afro_esc.h"

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdint.h>
#include <sys/ioctl.h>

#include <algorithm>  // used for min max functions

AfroESC::AfroESC(const int _i2c_handle, const int _i2c_address,
                 const int _pole_pairs)
    : available_(false), in_use_(false) {
  i2c_handle_ = _i2c_handle;
  i2c_address_ = _i2c_address;
  pole_pairs_ = _pole_pairs;
}

void AfroESC::Reset(int _i2c_handle, int _i2c_address) {
  i2c_handle_ = _i2c_handle;
  i2c_address_ = _i2c_address;
  available_ = false;
  in_use_ = false;
}

int AfroESC::WriteMotorSpeed() {
  int result;
  double speed = speed_;
  uint16_t speed_data = 0;
  if (speed < 0) {
    speed = -1 * speed_;
    speed_data += kInvertDirection;
  }

  speed_data += (int)(kMaxSpeed * speed);
  uint8_t data[] = {kRegSetSpeed, (uint8_t)(0xFF & (speed_data >> 8)),
                    (uint8_t)(speed_data & 0xFF)};
  struct i2c_msg msg = {(uint8_t)i2c_address_, 0, sizeof(data), data};
  struct i2c_rdwr_ioctl_data ioctl_data = {&msg, 1};
  if (ioctl(i2c_handle_, I2C_RDWR, &ioctl_data) != 1) {
    return false;
  }
  return true;
}
void AfroESC::SetMotorSpeed(double _speed) {
  speed_ = std::min(std::max(_speed, -1.0), 1.0);
}

int AfroESC::ReadMotorComCounter() { return 0; }
double AfroESC::GetMotorRevCounter() { return 0; }
int AfroESC::ReadBatteryAdc() { return 0; }
double AfroESC::GetBatteryVoltage() { return 0; }
int AfroESC::ReadTemperatureAdc() { return 0; }
int AfroESC::ReadId(int *id) {
  uint8_t buf[1];
  struct i2c_msg msgs[2];
  struct i2c_rdwr_ioctl_data msgset[1];
  msgs[0].addr = i2c_address_;
  msgs[0].flags = 0;
  msgs[0].len = sizeof(buf) / sizeof(buf[0]);
  msgs[0].buf = buf;

  msgs[1].addr = i2c_address_;
  msgs[1].flags = I2C_M_NOSTART | I2C_M_RD;
  msgs[1].len = sizeof(buf) / sizeof(buf[0]);
  msgs[1].buf = buf;

  msgset[0].msgs = msgs;
  msgset[0].nmsgs = sizeof(msgs) / sizeof(msgs[0]);
  if (ioctl(i2c_handle_, I2C_RDWR, msgset) < 0) {
    return -1;
  }
  *id = buf[0];
  return 0;
}
bool AfroESC::VerifyID() {
  int id;
  if (ReadId(&id) < 0) {
    SetAvailable(false);
    return false;
  }
  SetAvailable(kId == id);
  return (kId == id);
}
bool AfroESC::available() { return available_; }
void AfroESC::SetAvailable(bool available) { available_ = available; }
void AfroESC::SetIndex(int index) { index_ = index; }
int AfroESC::index() { return index_; }
void AfroESC::SetAddress(int _address) { i2c_address_ = _address; }
int AfroESC::address() { return i2c_address_; }
bool AfroESC::InUse() { return in_use_; }
void AfroESC::SetInUse(bool _in_use) { in_use_ = _in_use; }
