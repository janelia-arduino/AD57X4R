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


AD57X4R::AD57X4R()
{
}

AD57X4R::AD57X4R(const int cs_pin)
{
  setupCS(cs_pin);
  output_.header = 0;
}

void AD57X4R::init(const resolutions resolution, const output_ranges output_range)
{
  SPI.begin();
  resolution_ = resolution;
  setOutputRange(output_range, ALL);
  setPowerControlRegister(ALL);
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

void AD57X4R::analogWrite(const channels channel, const unsigned int value)
{
  setReadWrite(WRITE);
  setRegisterSelect(REGISTER_SELECT_DAC);
  setDACAddress(channel);
  setData(value);
  sendOutput();
  load();
}

void AD57X4R::analogWrite(const channels channel, const int value)
{
  setReadWrite(WRITE);
  setRegisterSelect(REGISTER_SELECT_DAC);
  setDACAddress(channel);
  setData(value);
  sendOutput();
  load();
}

void AD57X4R::analogWrite(const int pin, const unsigned int value)
{
  channels channel = pinToChannel(pin);
  analogWrite(channel, value);
}

void AD57X4R::analogWrite(const int pin, const int value)
{
  channels channel = pinToChannel(pin);
  analogWrite(channel, value);
}

unsigned int AD57X4R::getMaxDacValue()
{
  unsigned int max_dac_value = 65535;
  switch (resolution_)
  {
    case AD5724R:
      max_dac_value = 4095;
      break;
    case AD5734R:
      max_dac_value = 16383;
      break;
    case AD5754R:
      max_dac_value = 65535;
      break;
  }
  return max_dac_value;
}

void AD57X4R::setCSInvert()
{
  cs_invert_flag_ = true;
}

void AD57X4R::setCSNormal()
{
  cs_invert_flag_ = false;
}

// private
void AD57X4R::setupCS(const int cs_pin)
{
  cs_invert_flag_ = false;
  pinMode(cs_pin,OUTPUT);
  digitalWrite(cs_pin,HIGH);
  cs_pin_ = cs_pin;
}

void AD57X4R::spiBeginTransaction()
{
  SPI.beginTransaction(SPISettings(SPI_CLOCK,SPI_BIT_ORDER,SPI_MODE));
  csEnable();
}

void AD57X4R::spiEndTransaction()
{
  csDisable();
  SPI.endTransaction();
}

AD57X4R::channels AD57X4R::pinToChannel(const int pin)
{
  channels channel = ALL;
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
  return channel;
}

void AD57X4R::setHeader(const byte value, const byte bit_shift, const byte bit_count)
{
  byte bit_mask = 0;
  for (byte bit=0; bit<bit_count; bit++)
  {
    bitSet(bit_mask,(bit+bit_shift));
  }
  byte header = output_.header;
  header &= ~bit_mask;
  header |= value << bit_shift;
  output_.header = header;
}

void AD57X4R::setReadWrite(const byte value)
{
  setHeader(value,READ_WRITE_BIT_SHIFT,READ_WRITE_BIT_COUNT);
}

void AD57X4R::setRegisterSelect(const byte value)
{
  setHeader(value,REGISTER_SELECT_BIT_SHIFT,REGISTER_SELECT_BIT_COUNT);
}

void AD57X4R::setDACAddress(const channels channel)
{
  byte value = DAC_ADDRESS_ALL;
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
  setHeader(value,ADDRESS_BIT_SHIFT,ADDRESS_BIT_COUNT);
}

void AD57X4R::setControlAddress(const uint8_t address)
{
  setHeader(address,ADDRESS_BIT_SHIFT,ADDRESS_BIT_COUNT);
}

void AD57X4R::setNOP()
{
  output_.header = 0x18;
}

void AD57X4R::sendOutput()
{
  byte out_byte_header;
  byte out_byte_data_high;
  byte out_byte_data_low;

  spiBeginTransaction();

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
  SPI.transfer(out_byte_header);
  SPI.transfer(out_byte_data_high);
  SPI.transfer(out_byte_data_low);

  spiEndTransaction();
}

int AD57X4R::readInput()
{
  byte out_byte_header;
  byte out_byte_data_high;
  byte out_byte_data_low;
  byte in_byte_data_high;
  byte in_byte_data_low;
  int return_data;

  // Send NOP command in header
  setNOP();

  spiBeginTransaction();

  // Create and send command bytes
  out_byte_header = output_.header;
  out_byte_data_high = 0;
  out_byte_data_low = 0;
  SPI.transfer(out_byte_header);
  in_byte_data_high = SPI.transfer(out_byte_data_high);
  in_byte_data_low = SPI.transfer(out_byte_data_low);

  spiEndTransaction();

  // Fill return data
  return_data = ((int)in_byte_data_high << 8) | (int)in_byte_data_low;
  return return_data;
}

void AD57X4R::setData(const unsigned int value)
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

void AD57X4R::setData(const int value)
{
}

void AD57X4R::load()
{
  setReadWrite(WRITE);
  setRegisterSelect(REGISTER_SELECT_CONTROL);
  setControlAddress(CONTROL_ADDRESS_LOAD);
  sendOutput();
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

void AD57X4R::setOutputRange(const output_ranges output_range, const channels channel)
{
  setReadWrite(WRITE);
  setRegisterSelect(REGISTER_SELECT_OUTPUT_RANGE_SELECT);
  setDACAddress(channel);
  switch (output_range)
  {
    case UNIPOLAR_5V:
      unipolar_ = true;
      output_.data.unipolar = OUTPUT_RANGE_UNIPOLAR_5V;
      break;
    case UNIPOLAR_10V:
      unipolar_ = true;
      output_.data.unipolar = OUTPUT_RANGE_UNIPOLAR_10V;
      break;
    case BIPOLAR_5V:
      unipolar_ = false;
      output_.data.bipolar = OUTPUT_RANGE_BIPOLAR_5V;
      break;
    case BIPOLAR_10V:
      unipolar_ = false;
      output_.data.bipolar = OUTPUT_RANGE_BIPOLAR_10V;
      break;
  }
  sendOutput();
}

void AD57X4R::setPowerControlRegister(const channels channel)
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
