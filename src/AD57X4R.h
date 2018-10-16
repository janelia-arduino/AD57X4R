// ----------------------------------------------------------------------------
// AD57X4R.h
//
// Provides an SPI based interface to the AD57X4R
// Complete, Quad, 12-/14-/16-Bit, Serial Input,
// Unipolar/Bipolar Voltage Output DACs.
//
// Authors:
// Peter Polidoro peterpolidoro@gmail.com
// ----------------------------------------------------------------------------
#ifndef AD57X4R_H
#define AD57X4R_H
#include <SPI.h>


class AD57X4R
{
public:
  enum Resolution {AD5724R,
    AD5734R,
    AD5754R};
  enum Range {UNIPOLAR_5V,
    UNIPOLAR_10V,
    UNIPOLAR_10V8,
    BIPOLAR_5V,
    BIPOLAR_10V,
    BIPOLAR_10V8};

  AD57X4R();
  AD57X4R(size_t chip_select_pin);

  void setChipSelectPin(size_t pin);
  void setLoadDacPin(size_t pin);
  void setClearPin(size_t pin);

  void setup(Resolution resolution=AD5754R,
    uint8_t chip_count=1);

  uint8_t getChipCount();
  size_t getChannelCount();

  void setOutputRange(size_t channel,
    Range range);
  void setAllOutputRanges(Range range);

  long getAnalogValueMin(size_t channel);
  long getAnalogValueMax(size_t channel);

  void setAnalogValue(size_t channel,
    long analog_value);
  void setAllAnalogValues(long analog_value);
  void analogWrite(size_t channel,
    long analog_value);

  double getVoltageMin(size_t channel);
  double getVoltageMax(size_t channel);

  void setVoltage(size_t channel,
    double voltage);
  void setAllVoltages(double voltage);

  double analogValueToVoltage(size_t channel,
    long analog_value);
  long voltageToAnalogValue(size_t channel,
    double voltage);

  bool channelPoweredUp(size_t channel);
  bool referencePoweredUp(uint8_t chip);
  bool thermalShutdown(uint8_t chip);
  bool channelOverCurrent(size_t channel);

  void beginSimultaneousUpdate();
  void simultaneousUpdate();

private:
  size_t chip_select_pin_;
  size_t ldac_pin_;
  bool simultaneous_update_enabled_;
  size_t clr_pin_;

  int resolution_;

  const static uint8_t CHANNEL_COUNT_PER_CHIP = 4;

  const static int CHIP_ALL = -1;

  const static size_t CHIP_COUNT_MIN = 1;
  const static size_t CHIP_COUNT_MAX = 4;
  uint8_t chip_count_;

  const static size_t CHANNEL_MIN = 0;
  const static size_t CHANNEL_COUNT_MAX = CHIP_COUNT_MAX*CHANNEL_COUNT_PER_CHIP;

  const static uint32_t SPI_CLOCK = 8000000;
  const static uint8_t SPI_BIT_ORDER = MSBFIRST;
  const static uint8_t SPI_MODE = SPI_MODE2;

  // Datagrams
  const static uint8_t DATAGRAM_SIZE = 3;

  // Datagram
  union Datagram
  {
    struct Fields
    {
      uint32_t data : 16;
      uint32_t channel_address : 3;
      uint32_t reg : 3;
      uint32_t space0 : 1;
      uint32_t rw : 1;
      uint32_t space1 : 8;
    } fields;
    uint32_t uint32;
  };

  // Read/Write Bit
  const static uint8_t RW_READ = 1;
  const static uint8_t RW_WRITE = 0;

  // Register Bits
  const static uint8_t REGISTER_DAC = 0b000;
  const static uint8_t REGISTER_OUTPUT_RANGE = 0b001;
  const static uint8_t REGISTER_POWER_CONTROL = 0b010;
  const static uint8_t REGISTER_CONTROL = 0b011;

  // Channel Address Bits
  const static uint8_t CHANNEL_ADDRESS_A = 0b000;
  const static uint8_t CHANNEL_ADDRESS_B = 0b001;
  const static uint8_t CHANNEL_ADDRESS_C = 0b010;
  const static uint8_t CHANNEL_ADDRESS_D = 0b011;
  const static uint8_t CHANNEL_ADDRESS_ALL = 0b100;
  const static uint8_t CHANNEL_ADDRESS_POWER_CONTROL = 0b000;

  // Control Register Functions
  const static uint8_t CONTROL_ADDRESS_NOP = 0b000;
  const static uint8_t CONTROL_ADDRESS_CLEAR = 0b100;
  const static uint8_t CONTROL_ADDRESS_LOAD = 0b101;

  // Power Control Register
  const static uint16_t POWER_CONTROL_DAC_A = 0b1;
  const static uint16_t POWER_CONTROL_DAC_B = 0b10;
  const static uint16_t POWER_CONTROL_DAC_C = 0b100;
  const static uint16_t POWER_CONTROL_DAC_D = 0b1000;
  const static uint16_t POWER_CONTROL_REF = 0b10000;
  const static uint16_t POWER_CONTROL_THERMAL_SHUTDOWN = 0b100000;
  const static uint16_t POWER_CONTROL_OVERCURRENT_A = 0b10000000;
  const static uint16_t POWER_CONTROL_OVERCURRENT_B = 0b100000000;
  const static uint16_t POWER_CONTROL_OVERCURRENT_C = 0b1000000000;
  const static uint16_t POWER_CONTROL_OVERCURRENT_D = 0b10000000000;

  // Output Ranges
  const static uint8_t OUTPUT_RANGE_UNIPOLAR_5V = 0b000;
  const static uint8_t OUTPUT_RANGE_UNIPOLAR_10V = 0b001;
  const static uint8_t OUTPUT_RANGE_UNIPOLAR_10V8 = 0b010;
  const static uint8_t OUTPUT_RANGE_BIPOLAR_5V = 0b011;
  const static uint8_t OUTPUT_RANGE_BIPOLAR_10V = 0b100;
  const static uint8_t OUTPUT_RANGE_BIPOLAR_10V8 = 0b101;

  Range range_[CHANNEL_COUNT_MAX];

  void initialize();
  uint8_t channelToChip(size_t channel);
  uint8_t channelToChannelAddress(size_t channel);
  void enableClockSelect();
  void disableClockSelect();
  void spiBeginTransaction();
  void spiEndTransaction();
  void writeMosiDatagramToChip(int chip,
    Datagram mosi_datagram);
  Datagram readMisoDatagramFromChip(int chip);
  void powerUpAllDacs();
  void setOutputRangeToChip(int chip,
    uint8_t channel_address,
    Range range);
  void setAnalogValueToChip(int chip,
    uint8_t channel_address,
    long data);
  void load(int chip);
  uint16_t readPowerControlRegister(uint8_t chip);
  bool rangeIsBipolar(Range range);
};
#endif
