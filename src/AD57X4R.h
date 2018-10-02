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
  enum resolutions {AD5724R, AD5734R, AD5754R};
  enum output_ranges {UNIPOLAR_5V, UNIPOLAR_10V, BIPOLAR_5V, BIPOLAR_10V};
  enum channels {A, B, C, D, ALL};

  AD57X4R();
  AD57X4R(const size_t chip_select_pin);

  void setChipSelectPin(const size_t pin);
  void setLoadDacPin(const size_t pin);
  void setClearPin(const size_t pin);
  void init(const resolutions resolution=AD5754R, const output_ranges output_range=UNIPOLAR_5V);

  int readPowerControlRegister();

  void analogWrite(const channels channel, const unsigned int value);
  void analogWrite(const channels channel, const int value);
  void analogWrite(const size_t pin, const unsigned int value);
  void analogWrite(const size_t pin, const int value);

  unsigned int getMaxDacValue();

  void setCSInvert();
  void setCSNormal();
private:
  // Read/Write Bit
  const static uint8_t READ = 1;
  const static uint8_t WRITE = 0;
  const static uint8_t READ_WRITE_BIT_SHIFT = 7;
  const static uint8_t READ_WRITE_BIT_COUNT = 1;

  // Register Select Bits
  const static uint8_t REGISTER_SELECT_DAC = 0b000;
  const static uint8_t REGISTER_SELECT_OUTPUT_RANGE_SELECT = 0b001;
  const static uint8_t REGISTER_SELECT_POWER_CONTROL = 0b010;
  const static uint8_t REGISTER_SELECT_CONTROL = 0b011;
  const static uint8_t REGISTER_SELECT_BIT_SHIFT = 3;
  const static uint8_t REGISTER_SELECT_BIT_COUNT = 3;

  const static uint8_t ADDRESS_BIT_SHIFT =  0;
  const static uint8_t ADDRESS_BIT_COUNT = 3;

  // DAC Address Bits
  const static uint8_t DAC_ADDRESS_A = 0b000;
  const static uint8_t DAC_ADDRESS_B = 0b001;
  const static uint8_t DAC_ADDRESS_C = 0b010;
  const static uint8_t DAC_ADDRESS_D = 0b011;
  const static uint8_t DAC_ADDRESS_ALL = 0b100;

  // Control Register Functions
  const static uint8_t CONTROL_ADDRESS_NOP = 0b000;
  const static uint8_t CONTROL_ADDRESS_CLEAR = 0b100;
  const static uint8_t CONTROL_ADDRESS_LOAD = 0b101;

  // Output Ranges
  const static uint8_t OUTPUT_RANGE_UNIPOLAR_5V = 0b000;
  const static uint8_t OUTPUT_RANGE_UNIPOLAR_10V = 0b001;
  const static uint8_t OUTPUT_RANGE_BIPOLAR_5V = 0b011;
  const static uint8_t OUTPUT_RANGE_BIPOLAR_10V = 0b100;

  const static uint32_t SPI_CLOCK = 8000000;
  const static uint8_t SPI_BIT_ORDER = MSBFIRST;
  const static uint8_t SPI_MODE = SPI_MODE2;

  int resolution_;

  size_t cs_pin_;
  size_t ldac_pin_;
  size_t clr_pin_;

  struct shift_register
  {
    byte header;
    union
    {
      unsigned int unipolar;
      int bipolar;
    } data;
  } output_;
  struct shift_register input_;
  boolean unipolar_;
  boolean cs_invert_flag_;

  void spiBeginTransaction();
  void spiEndTransaction();
  channels pinToChannel(const size_t pin);
  void setHeader(const byte value, const byte bit_shift, const byte bit_count);
  void setReadWrite(const byte value);
  void setRegisterSelect(const byte value);
  void setDACAddress(const channels channel);
  void setControlAddress(const uint8_t address);
  void setNOP();
  void sendOutput();
  int readInput();
  void setData(const unsigned int value);
  void setData(const int value);
  void load();
  void csEnable();
  void csDisable();
  void setOutputRange(const output_ranges output_range, const channels channel);
  void setPowerControlRegister(const channels channel);
};
#endif
