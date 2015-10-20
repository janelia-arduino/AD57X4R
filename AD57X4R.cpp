// ----------------------------------------------------------------------------
// AD57X4R.cpp
//
// Provides an SPI based interface to the AD57X4R
// Complete, Quad, 12-/14-/16-Bit, Serial Input,
// Unipolar/Bipolar Voltage Output DACs.
//
// Authors:
// Peter Polidoro polidorop@janelia.hhmi.org
// ----------------------------------------------------------------------------
#include "AD57X4R.h"


// Read/Write Bit
#define READ                 1
#define WRITE                0
#define READ_WRITE_BIT_SHIFT 7
#define READ_WRITE_BIT_COUNT 1

// Register Select Bits
#define REGISTER_SELECT_DAC                 0b000
#define REGISTER_SELECT_OUTPUT_RANGE_SELECT 0b001
#define REGISTER_SELECT_POWER_CONTROL       0b010
#define REGISTER_SELECT_CONTROL             0b011
#define REGISTER_SELECT_BIT_SHIFT           3
#define REGISTER_SELECT_BIT_COUNT           3

// DAC Address Bits
#define DAC_ADDRESS_A         0b000
#define DAC_ADDRESS_B         0b001
#define DAC_ADDRESS_C         0b010
#define DAC_ADDRESS_D         0b011
#define DAC_ADDRESS_ALL       0b100
#define DAC_ADDRESS_BIT_SHIFT 0
#define DAC_ADDRESS_BIT_COUNT 3

// Output Ranges
#define OUTPUT_RANGE_UNIPOLAR_5V  0b000
#define OUTPUT_RANGE_UNIPOLAR_10V 0b001
#define OUTPUT_RANGE_BIPOLAR_5V   0b011
#define OUTPUT_RANGE_BIPOLAR_10V  0b100

AD57X4R::AD57X4R()
{
}

AD57X4R::AD57X4R(int cs_pin)
{
  setupCS(cs_pin);
  output_.header = 0;
}

void AD57X4R::spiBegin()
{
  SPI.setDataMode(SPI_MODE2);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.begin();
}

void AD57X4R::init(resolutions resolution, output_ranges output_range, boolean spi_reset)
{
  spiBegin();
  resolution_ = resolution;
  setOutputRange(output_range, ALL);
  setPowerControlRegister(ALL);
  spi_reset_ = spi_reset;
}

void AD57X4R::setupCS(int cs_pin)
{
  cs_invert_flag_ = false;
  pinMode(cs_pin,OUTPUT);
  digitalWrite(cs_pin,HIGH);
  cs_pin_ = cs_pin;
}

void AD57X4R::setHeader(byte value, byte bit_shift, byte bit_count)
{
  byte bit_mask = 0;
  for (byte bit=0; bit<bit_count; bit++)
  {
    bitSet(bit_mask,(bit+bit_shift));
  }
  byte header = output_.header;
  header &= ~bit_mask;
  value = value << bit_shift;
  header |= value;
  output_.header = header;
}

void AD57X4R::setReadWrite(byte value)
{
  setHeader(value,READ_WRITE_BIT_SHIFT,READ_WRITE_BIT_COUNT);
}

void AD57X4R::setRegisterSelect(byte value)
{
  setHeader(value,REGISTER_SELECT_BIT_SHIFT,REGISTER_SELECT_BIT_COUNT);
}

void AD57X4R::setDACAddress(channels channel)
{
  byte value;
  switch (channel)
  {
    case A:
      value = DAC_ADDRESS_A;
      break;
    case B:
      value = DAC_ADDRESS_B;
      break;
    case C:
      value = DAC_ADDRESS_C;
      break;
    case D:
      value = DAC_ADDRESS_D;
      break;
    case ALL:
      value = DAC_ADDRESS_ALL;
      break;
  }
  setHeader(value,DAC_ADDRESS_BIT_SHIFT,DAC_ADDRESS_BIT_COUNT);
}

void AD57X4R::setNOP()
{
  output_.header = 0x18;
}

void AD57X4R::csEnable()
{
  if (cs_invert_flag_ == false)
  {
    digitalWrite(cs_pin_,LOW);
  }
  else
  {
    digitalWrite(cs_pin_,HIGH);
  }
}

void AD57X4R::csDisable()
{
  if (cs_invert_flag_ == false)
  {
    digitalWrite(cs_pin_,HIGH);
  }
  else
  {
    digitalWrite(cs_pin_,LOW);
  }
}

void AD57X4R::sendOutput()
{
  byte out_byte_header;
  byte out_byte_data_high;
  byte out_byte_data_low;
  byte return_byte;

  if (spi_reset_)
  {
    spiBegin();
  }

  // Enable SPI communication
  csEnable();

  // Create and send command bytes
  out_byte_header = output_.header;
  if (unipolar_)
  {
    unsigned int data;
    data = output_.data.unipolar;
    output_.data.unipolar = 0;
    out_byte_data_high = highByte(data);
    out_byte_data_low = lowByte(data);
  }
  else
  {
    int data;
    data = output_.data.bipolar;
    output_.data.bipolar = 0;
    out_byte_data_high = highByte(data);
    out_byte_data_low = lowByte(data);
  }
  return_byte = SPI.transfer(out_byte_header);
  return_byte = SPI.transfer(out_byte_data_high);
  return_byte = SPI.transfer(out_byte_data_low);
  // Serial << "out_byte_header = " << _BIN(out_byte_header) << endl;
  // Serial << "out_byte_data_high = " << _BIN(out_byte_data_high) << endl;
  // Serial << "out_byte_data_low = " << _BIN(out_byte_data_low) << endl;

  // Disable SPI communication
  csDisable();
}

int AD57X4R::readInput()
{
  byte out_byte_header;
  byte out_byte_data_high;
  byte out_byte_data_low;
  byte in_byte_header;
  byte in_byte_data_high;
  byte in_byte_data_low;
  int return_data;

  // Send NOP command in header
  setNOP();

  if (spi_reset_)
  {
    spiBegin();
  }

  // Enable SPI communication
  csEnable();

  // Create and send command bytes
  out_byte_header = output_.header;
  out_byte_data_high = 0;
  out_byte_data_low = 0;
  in_byte_header = SPI.transfer(out_byte_header);
  in_byte_data_high = SPI.transfer(out_byte_data_high);
  in_byte_data_low = SPI.transfer(out_byte_data_low);
  // Serial << "in_byte_header = " << _BIN(in_byte_header) << endl;
  // Serial << "in_byte_data_high = " << _BIN(in_byte_data_high) << endl;
  // Serial << "in_byte_data_low = " << _BIN(in_byte_data_low) << endl;

  // Disable SPI communication
  csDisable();

  // Fill return data
  return_data = ((int)in_byte_data_high << 8) | (int)in_byte_data_low;
  return return_data;
}

void AD57X4R::setData(unsigned int value)
{
  switch (resolution_)
  {
    case AD5754R:
      output_.data.unipolar = value;
      break;
    case AD5734R:
      output_.data.unipolar = value << 2;
      break;
    case AD5724R:
      output_.data.unipolar = value << 4;
      break;
  }
}

void AD57X4R::setData(int value)
{
}

void AD57X4R::setOutputRange(output_ranges output_range, channels channel)
{
  setReadWrite(WRITE);
  setRegisterSelect(REGISTER_SELECT_OUTPUT_RANGE_SELECT);
  setDACAddress(channel);
  switch (output_range)
  {
    case UNIPOLAR_5V:
      unipolar_ = true;
      output_.data.unipolar = 0b000;
      break;
    case UNIPOLAR_10V:
      unipolar_ = true;
      output_.data.unipolar = 0b001;
      break;
    case BIPOLAR_5V:
      unipolar_ = false;
      output_.data.bipolar = 0b011;
      break;
    case BIPOLAR_10V:
      unipolar_ = false;
      output_.data.bipolar = 0b101;
      break;
  }
  sendOutput();
}

void AD57X4R::setPowerControlRegister(channels channel)
{
  setReadWrite(WRITE);
  setRegisterSelect(REGISTER_SELECT_POWER_CONTROL);
  setDACAddress(A);
  bool unipolar_previous = unipolar_;
  unipolar_ = true;
  // place internal reference in normal operating mode
  output_.data.unipolar = 0b10000;
  // power up DACs
  switch (channel)
  {
    case A:
      output_.data.unipolar |= 0b0001;
      break;
    case B:
      output_.data.unipolar |= 0b0010;
      break;
    case C:
      output_.data.unipolar |= 0b0100;
      break;
    case D:
      output_.data.unipolar |= 0b1000;
      break;
    case ALL:
      output_.data.unipolar |= 0b1111;
      break;
  }
  sendOutput();
  unipolar_ = unipolar_previous;
}

int AD57X4R::readPowerControlRegister()
{
  int return_data;
  setReadWrite(READ);
  setRegisterSelect(REGISTER_SELECT_POWER_CONTROL);
  setDACAddress(A);
  sendOutput();
  return_data = readInput();
  return return_data;
}

void AD57X4R::analogWrite(channels channel, unsigned int value)
{
  if (unipolar_)
  {
    setReadWrite(WRITE);
    setRegisterSelect(REGISTER_SELECT_DAC);
    setDACAddress(channel);
    setData(value);
    sendOutput();
  }
}

void AD57X4R::analogWrite(channels channel, int value)
{
  if (unipolar_)
  {
    analogWrite(channel,(unsigned int)value);
  }
}

void AD57X4R::analogWrite(int pin, unsigned int value)
{
  channels channel;
  // Unnecessary and way too much code, but very explicit
  switch (pin)
  {
    case 0:
      channel = A;
      break;
    case 1:
      channel = B;
      break;
    case 2:
      channel = C;
      break;
    case 3:
      channel = D;
      break;
  }
  analogWrite(channel, value);
}

void AD57X4R::analogWrite(int pin, int value)
{
  channels channel;
  // Unnecessary and way too much code, but very explicit
  switch (pin)
  {
    case 0:
      channel = A;
      break;
    case 1:
      channel = B;
      break;
    case 2:
      channel = C;
      break;
    case 3:
      channel = D;
      break;
  }
  analogWrite(channel, value);
}

unsigned int AD57X4R::getMaxDacValue()
{
  switch (resolution_)
  {
    case AD5724R:
      return 4095;
      break;
    case AD5734R:
      return 16383;
      break;
    case AD5754R:
      return 65535;
      break;
  }
}

void AD57X4R::setCSInvert()
{
  cs_invert_flag_ = true;
}

void AD57X4R::setCSNormal()
{
  cs_invert_flag_ = false;
}
