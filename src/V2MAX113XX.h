// © Kay Sievers <kay@versioduo.com>, 2020-2022
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>
#include <SPI.h>

namespace V2MAX113XX {
struct Field {
  uint8_t position;
  uint8_t size;
};

enum class Variant { MAX11300, MAX11311 };

namespace Address {
  enum {
    TemperatureData  = 0x08,
    Control          = 0x10,
    TemperatureMax   = 0x19,
    PortConfig       = 0x20,
    AnalogInputData  = 0x40,
    AnalogOutputData = 0x60
  };
};

namespace Control {
  namespace AnalogInput {
    enum Value { Idle, SingleSweep, SingleConversion, ContinuousSweep };
    static constexpr Field field{.position{0}, .size{2}};
  };

  namespace AnalogOutput {
    enum Value { SequentialUpdate, ImmediateUpdate, SameData1, SameData2 };
    static constexpr Field field{.position{2}, .size{2}};
  };

  namespace SampleRate {
    static constexpr Field SampleRate{.position{4}, .size{2}};
  };

  namespace InternalReference {
    static constexpr Field field{.position{6}, .size{1}};
  };

  namespace ThermalShutdown {
    static constexpr Field field{.position{7}, .size{1}};
  };

  namespace TemperatureMonitor {
    enum Value { Internal = 1 << 0, External1 = 1 << 1, External2 = 1 << 2 };
    static constexpr Field field{.position{8}, .size{3}};
  };

  namespace Reset {
    static constexpr Field field{.position{14}, .size{1}};
  };
};

namespace PortConfig {
  namespace Pair {
    // Associated port, needs separate configuration.
    static constexpr Field field{.position{0}, .size{5}};
  };

  namespace Samples {
    // 2^n samples.
    static constexpr Field field{.position{5}, .size{3}};
  };

  namespace Voltage {
    enum Value { None, p10, n5_p5, n10 };
    static constexpr Field field{.position{8}, .size{3}};
  };

  namespace Mode {
    enum Value {
      HighImpedance,
      DigitalInput,
      LevelTranslator,
      DigitalOutput,
      DigitalOutputLevelTranslator,
      AnalogOutput,
      AnalogOutputMonitor,
      AnalogInputPositive,
      AnalogInputDifferentialPositive,
      AnalogInputDifferentialNegative,
      AnalogInputDifferentialNegativeShift,
      AnalogSwitch,
      AnalogSwitchClosed
    };
    static constexpr Field field{.position{12}, .size{4}};
  };
};

class Driver {
public:
  constexpr Driver(Variant variant, SPIClass *spi, uint8_t pin) : _variant(variant), _spi(spi), _pin(pin) {}
  void begin();
  void reset();
  float readTemperature();
  void configurePort(uint8_t port,
                     PortConfig::Mode::Value mode,
                     PortConfig::Voltage::Value voltage,
                     uint8_t samples = 0);
  float readPort(uint8_t port);
  void writePort(uint8_t port, float volt);

private:
  Variant _variant;
  SPIClass *_spi;
  const uint8_t _pin;
  struct {
    PortConfig::Mode::Value mode;
    PortConfig::Voltage::Value voltage;
  } _ports[20]{};

  void readWrite(uint8_t *data, uint8_t length);
  uint16_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint16_t value);
  uint8_t getPortOffset(uint8_t port);
};
};
