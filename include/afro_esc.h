#pragma once

#include <stdint.h>

#include "esc.h"
class AfroESC : public ESCBase {
 private:
  static constexpr int kRegSetSpeed = 0x00;
  static constexpr int kRegGetSpeed = 0x02;
  static constexpr int kRegGetVbat = 0x04;
  static constexpr int kRegGetTemp = 0x06;
  static constexpr int kRegGetId = 0x08;
  static constexpr int kId = 0xAB;
  static constexpr int kMaxSpeed = INT16_MAX;
  static constexpr int kInvertDirection = (1 << 15);
  static constexpr double kAdcVref = 5.0;
  static constexpr int kAdcResolution = 1024;
  static constexpr double kAdcVbatVoltageDivider = 6.45;
  static constexpr double kAdcVbatScaler =
      kAdcVref / kAdcResolution * kAdcVbatVoltageDivider;

  int i2c_handle_;
  int i2c_address_;
  int pole_pairs_;

 public:
  AfroESC(const int _i2c_handle, const int _i2c_address,
          const int _pole_pairs = 6);
  bool SetMotorSpeed(double speed);
  int GetMotorComCounter();
  double GetMotorRevCounter();
  int GetBatteryAdc();
  double GetBatteryVoltage();
  int GetTemperatureAdc();
  int GetID();
  bool VerifyID();
};
