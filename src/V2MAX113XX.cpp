// © Kay Sievers <kay@versioduo.com>, 2020-2022
// SPDX-License-Identifier: Apache-2.0

#include "V2MAX113XX.h"

static uint16_t getField(uint16_t data, V2MAX113XX::Field field) {
  const uint16_t mask = ((1 << field.size) - 1);
  return (data >> field.position) & mask;
}

static void setField(uint16_t &data, V2MAX113XX::Field field, uint16_t value) {
  const uint16_t mask = ((1 << field.size) - 1);
  data &= ~(mask << field.position);
  data |= (value & mask) << field.position;
}

void V2MAX113XX::Driver::readWrite(uint8_t *data, uint8_t length) {
  _spi->beginTransaction(SPISettings(10 * 1000 * 1000, MSBFIRST, SPI_MODE0));
  digitalWrite(_pin, LOW);
  _spi->transfer(data, length);
  digitalWrite(_pin, HIGH);
  _spi->endTransaction();
}

uint16_t V2MAX113XX::Driver::readRegister(uint8_t address) {
  uint8_t data[3]{(uint8_t)(address << 1 | 1), 0xff, 0xff};
  readWrite(data, sizeof(data));
  return data[1] << 8 | data[2];
}

void V2MAX113XX::Driver::writeRegister(uint8_t address, uint16_t value) {
  uint8_t data[3]{(uint8_t)(address << 1), (uint8_t)(value >> 8), (uint8_t)(value & 0xff)};
  readWrite(data, sizeof(data));
}

void V2MAX113XX::Driver::begin() {
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, HIGH);
  reset();
}

void V2MAX113XX::Driver::reset() {
  {
    uint16_t value = 0;
    setField(value, Control::Reset::field, 1);
    writeRegister(Address::Control, value);
    delay(1);
  }
  {
    uint16_t value = 0;
    setField(value, Control::AnalogInput::field, Control::AnalogInput::ContinuousSweep);
    setField(value, Control::TemperatureMonitor::field, Control::TemperatureMonitor::Internal);
    setField(value, Control::ThermalShutdown::field, 1);
    setField(value, Control::InternalReference::field, 1);
    writeRegister(Address::Control, value);
    delayMicroseconds(200);
  }
  {
    // Set the shutdown temperature threshold to 80°C.
    const int16_t value = 80 * 8;
    writeRegister(Address::TemperatureMax, value >> 4);
  }

  memset(_ports, 0, sizeof(_ports));
}

float V2MAX113XX::Driver::readTemperature() {
  // Shift to 16 bit intermediate value, to handle signdness.
  const int16_t value = (int16_t)(readRegister(Address::TemperatureData) << 4);
  return (float)(value >> 4) / 8.f;
}

uint8_t V2MAX113XX::Driver::getPortOffset(uint8_t port) {
  switch (_variant) {
    case Variant::MAX11300:
      return port;

    case Variant::MAX11311:
      if (port > 5)
        return port + 2 + 3;

      return port + 2;
  }

  return 0;
}

void V2MAX113XX::Driver::configurePort(uint8_t port,
                                       PortConfig::Mode::Value mode,
                                       PortConfig::Voltage::Value voltage,
                                       uint8_t samples) {
  _ports[port].mode    = mode;
  _ports[port].voltage = voltage;
  writePort(port, 0);

  uint16_t value = 0;
  setField(value, PortConfig::Samples::field, samples);
  setField(value, PortConfig::Voltage::field, voltage);
  setField(value, PortConfig::Mode::field, mode);
  writeRegister(Address::PortConfig + getPortOffset(port), value);
  delay(1);
}

float V2MAX113XX::Driver::readPort(uint8_t port) {
  if (_ports[port].mode != PortConfig::Mode::AnalogInputPositive)
    return 0;

  const float fraction = (float)readRegister(Address::AnalogInputData + getPortOffset(port)) / 4095.f;

  switch (_ports[port].voltage) {
    case PortConfig::Voltage::p10:
      return 10.f * fraction;

    case PortConfig::Voltage::n5_p5:
      return (10.f * fraction) - 5.f;
  }

  return 0;
}

void V2MAX113XX::Driver::writePort(uint8_t port, const float volt) {
  if (_ports[port].mode != PortConfig::Mode::AnalogOutput)
    return;

  float fraction = 0;

  switch (_ports[port].voltage) {
    case PortConfig::Voltage::p10:
      fraction = volt / 10.f;
      break;

    case PortConfig::Voltage::n5_p5:
      fraction = (volt + 5.f) / 10.f;
      break;
  }

  writeRegister(Address::AnalogOutputData + getPortOffset(port), 4095.f * fraction);
}
