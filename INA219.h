// INA219 driver for the Waveshare UPS Module 3S V2 (3S 18650 pack monitor).
//
// Ported from the Python driver used on my RobiOneKenobi robot (UPS_Module_3S_V2/INA219.py)
// — same registers, same 16V/5A calibration, same maths, so readings match it 1:1.
//
// Written for the Uno Q build of this chassis, unchanged except this note. The driver
// itself is board-agnostic — it only ever talks to a `TwoWire` — so the copy is verbatim.
//
// On the Uno R4 WiFi the module's SDA/SCL land on **A4/A5**, which is plain `Wire`. Those
// two pins are kept clear of the motor pin map for exactly this reason; see the MOTORS[]
// note in r4_mecanum_base.ino before reassigning them. (On the Uno Q the same sensor sits on
// the Uno Q's D20/D21, also plain `Wire`.)
//
// Address 0x41 (ADDR pulled high on the module).
//
// Calibration: set_calibration_16V_5A — 0.01 ohm shunt, VBUS 16 V, gain /2 (80 mV),
// 12-bit/32-sample ADC, shunt+bus continuous. Current LSB 0.1524 mA, power LSB
// 0.003048 W, cal register 26868. The calibration register is re-written before each
// shunt/bus/power read, exactly as the Python driver does (it can be clobbered by a
// bus glitch, and a silently wrong cal would read as a fake low battery).
#pragma once

#include <Arduino.h>
#include <Wire.h>

class INA219 {
public:
  static const uint8_t DEFAULT_ADDR = 0x41;

  // Probe + configure. Returns false if the chip doesn't ACK (module off / not wired).
  bool begin(uint8_t addr = DEFAULT_ADDR, TwoWire *wire = &Wire) {
    _addr = addr;
    _wire = wire;
    _wire->begin();
    return setCalibration_16V_5A();
  }

  // Load-side bus voltage, volts. 4 mV/LSB, top 13 bits.
  bool busVoltage_V(float &out) {
    if (!write16(REG_CALIBRATION, CAL_VALUE)) return false;
    uint16_t raw;
    if (!read16(REG_BUSVOLTAGE, raw)) return false;
    out = (float)(raw >> 3) * 0.004f;
    return true;
  }

  // Voltage across the shunt, millivolts (signed). 10 uV/LSB.
  bool shuntVoltage_mV(float &out) {
    if (!write16(REG_CALIBRATION, CAL_VALUE)) return false;
    uint16_t raw;
    if (!read16(REG_SHUNTVOLTAGE, raw)) return false;
    out = (float)signed16(raw) * 0.01f;
    return true;
  }

  // Load current, milliamps (signed: negative while the pack is charging).
  bool current_mA(float &out) {
    uint16_t raw;
    if (!read16(REG_CURRENT, raw)) return false;
    out = (float)signed16(raw) * CURRENT_LSB_MA;
    return true;
  }

  bool power_W(float &out) {
    if (!write16(REG_CALIBRATION, CAL_VALUE)) return false;
    uint16_t raw;
    if (!read16(REG_POWER, raw)) return false;
    out = (float)signed16(raw) * POWER_LSB_W;
    return true;
  }

  // One shot of everything. packV is the *supply-side* voltage (bus + shunt drop) —
  // that's the actual pack terminal voltage, which is what the thresholds want.
  bool sample(float &packV, float &busV, float &mA, float &W) {
    float shunt_mV;
    if (!busVoltage_V(busV)) return false;
    if (!shuntVoltage_mV(shunt_mV)) return false;
    if (!current_mA(mA)) return false;
    if (!power_W(W)) return false;
    packV = busV + shunt_mV / 1000.0f;
    return true;
  }

private:
  // Register map (INA219 datasheet; identical to the Python driver's).
  static const uint8_t REG_CONFIG       = 0x00;
  static const uint8_t REG_SHUNTVOLTAGE = 0x01;
  static const uint8_t REG_BUSVOLTAGE   = 0x02;
  static const uint8_t REG_POWER        = 0x03;
  static const uint8_t REG_CURRENT      = 0x04;
  static const uint8_t REG_CALIBRATION  = 0x05;

  // set_calibration_16V_5A(): Cal = 0.04096 / (current_LSB * Rshunt)
  //                               = 0.04096 / (0.0001524 A * 0.01 ohm) ~= 26868
  static const uint16_t CAL_VALUE      = 26868;
  static constexpr float CURRENT_LSB_MA = 0.1524f;
  static constexpr float POWER_LSB_W    = 0.003048f;

  // CONFIG = BRNG(16V)<<13 | PG(/2, 80mV)<<11 | BADC(12bit,32S)<<7 | SADC(12bit,32S)<<3 | MODE(sandb cont)
  //        = 0<<13 | 1<<11 | 0x0D<<7 | 0x0D<<3 | 7 = 0x0EEF
  static const uint16_t CONFIG_16V_5A  = 0x0EEF;

  bool setCalibration_16V_5A() {
    if (!write16(REG_CALIBRATION, CAL_VALUE)) return false;
    return write16(REG_CONFIG, CONFIG_16V_5A);
  }

  static int16_t signed16(uint16_t raw) { return (int16_t)raw; }   // two's complement

  bool write16(uint8_t reg, uint16_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write((uint8_t)(val >> 8));
    _wire->write((uint8_t)(val & 0xFF));
    return _wire->endTransmission() == 0;
  }

  bool read16(uint8_t reg, uint16_t &out) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission() != 0) return false;
    if (_wire->requestFrom(_addr, (size_t)2) != 2) return false;
    uint8_t hi = (uint8_t)_wire->read();
    uint8_t lo = (uint8_t)_wire->read();
    out = ((uint16_t)hi << 8) | lo;
    return true;
  }

  uint8_t   _addr = DEFAULT_ADDR;
  TwoWire  *_wire = &Wire;
};
