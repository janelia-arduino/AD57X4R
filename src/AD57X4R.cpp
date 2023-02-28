// ----------------------------------------------------------------------------
// AD57X4R.cpp
//
// Provides an SPI based interface to the AD57X4R
// Complete, Quad, 12-/14-/16-Bit, Serial Input,
// Unipolar/Bipolar Voltage Output DACs.
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "AD57X4R.h"


AD57X4R::AD57X4R()
{
  initialize();
}

AD57X4R::AD57X4R(size_t chip_select_pin)
{
  initialize();
  setChipSelectPin(chip_select_pin);
}

void AD57X4R::setChipSelectPin(size_t pin)
{
  pinMode(pin,OUTPUT);
  digitalWrite(pin,HIGH);
  chip_select_pin_ = pin;
}

void AD57X4R::setLoadDacPin(size_t pin)
{
  pinMode(pin,OUTPUT);
  digitalWrite(pin,LOW);
  ldac_pin_ = pin;
  simultaneous_update_enabled_ = true;
}

void AD57X4R::setClearPin(size_t pin)
{
  pinMode(pin,OUTPUT);
  digitalWrite(pin,HIGH);
  clr_pin_ = pin;
}

void AD57X4R::setup(Resolution resolution,
  uint8_t chip_count)
{
  if ((chip_count >= CHIP_COUNT_MIN) && (chip_count <= CHIP_COUNT_MAX))
  {
    chip_count_ = chip_count;
  }
  else
  {
    chip_count_ = CHIP_COUNT_MIN;
  }
  resolution_ = resolution;
  SPI.begin();
  powerUpAllDacs();
  setAllOutputRanges(UNIPOLAR_5V);
  setAllAnalogValues(0);
}

uint8_t AD57X4R::getChipCount()
{
  return chip_count_;
}

size_t AD57X4R::getChannelCount()
{
  return chip_count_*CHANNEL_COUNT_PER_CHIP;
}

void AD57X4R::setOutputRange(size_t channel,
  Range range)
{
  size_t channel_constrained = constrain(channel,
    CHANNEL_MIN,
    getChannelCount()-1);
  uint8_t chip = channelToChip(channel_constrained);
  uint8_t channel_address = channelToChannelAddress(channel_constrained);
  range_[channel_constrained] = range;
  setOutputRangeOnChip(chip,channel_address,range);
}

void AD57X4R::setAllOutputRanges(Range range)
{
  uint8_t chip = CHIP_ALL;
  uint8_t channel_address = CHANNEL_ADDRESS_ALL;
  for (size_t channel=0; channel<getChannelCount(); ++channel)
  {
    range_[channel] = range;
  }
  setOutputRangeOnChip(chip,channel_address,range);
}

long AD57X4R::getAnalogValueMin(size_t channel)
{
  long analog_value_min = 0;
  if (channel >= getChannelCount())
  {
    return analog_value_min;
  }
  if (rangeIsBipolar(range_[channel]))
  {
    switch (resolution_)
    {
      case AD5724R:
        analog_value_min = -2048;
        break;
      case AD5734R:
        analog_value_min = -8192;
        break;
      case AD5754R:
        analog_value_min = -32768;
        break;
    }
  }
  return analog_value_min;
}

long AD57X4R::getAnalogValueMax(size_t channel)
{
  long analog_value_max = 0;
  if (channel >= getChannelCount())
  {
    return analog_value_max;
  }
  if (rangeIsBipolar(range_[channel]))
  {
    switch (resolution_)
    {
      case AD5724R:
        analog_value_max = 2047;
        break;
      case AD5734R:
        analog_value_max = 8191;
        break;
      case AD5754R:
        analog_value_max = 32767;
        break;
    }
  }
  else
  {
    switch (resolution_)
    {
      case AD5724R:
        analog_value_max = 4095;
        break;
      case AD5734R:
        analog_value_max = 16383;
        break;
      case AD5754R:
        analog_value_max = 65535;
        break;
    }
  }
  return analog_value_max;
}

void AD57X4R::setAnalogValue(size_t channel,
  long analog_value)
{
  size_t channel_constrained = constrain(channel,
    CHANNEL_MIN,
    getChannelCount()-1);
  long analog_value_constrained = constrain(analog_value,
    getAnalogValueMin(channel_constrained),
    getAnalogValueMax(channel_constrained));
  uint8_t chip = channelToChip(channel_constrained);
  uint8_t channel_address = channelToChannelAddress(channel_constrained);
  setAnalogValueOnChip(chip,channel_address,analog_value_constrained);
}

void AD57X4R::setAllAnalogValues(long analog_value)
{
  for (size_t channel=0; channel<getChannelCount(); ++channel)
  {
    setAnalogValue(channel,analog_value);
  }
}

void AD57X4R::analogWrite(size_t channel,
  long analog_value)
{
  setAnalogValue(channel,analog_value);
}

double AD57X4R::getVoltageMin(size_t channel)
{
  double voltage_min = 0.0;
  if (channel >= getChannelCount())
  {
    return voltage_min;
  }
  switch (range_[channel])
  {
    case UNIPOLAR_5V:
      voltage_min = 0.0;
      break;
    case UNIPOLAR_10V:
      voltage_min = 0.0;
      break;
    case UNIPOLAR_10V8:
      voltage_min = 0.0;
      break;
    case BIPOLAR_5V:
      voltage_min = -5.0;
      break;
    case BIPOLAR_10V:
      voltage_min = -10.0;
      break;
    case BIPOLAR_10V8:
      voltage_min = -10.8;
      break;
    default:
      voltage_min = 0.0;
      break;
  }
  return voltage_min;
}

double AD57X4R::getVoltageMax(size_t channel)
{
  double voltage_max = 0.0;
  if (channel >= getChannelCount())
  {
    return voltage_max;
  }
  switch (range_[channel])
  {
    case UNIPOLAR_5V:
      voltage_max = 5.0;
      break;
    case UNIPOLAR_10V:
      voltage_max = 10.0;
      break;
    case UNIPOLAR_10V8:
      voltage_max = 10.8;
      break;
    case BIPOLAR_5V:
      voltage_max = 5.0;
      break;
    case BIPOLAR_10V:
      voltage_max = 10.0;
      break;
    case BIPOLAR_10V8:
      voltage_max = 10.8;
      break;
    default:
      voltage_max = 0.0;
      break;
  }
  return voltage_max;
}

void AD57X4R::setVoltage(size_t channel,
  double voltage)
{
  // Wastes resolution, need to change algorithm
  size_t channel_constrained = constrain(channel,
    CHANNEL_MIN,
    getChannelCount()-1);
  uint8_t chip = channelToChip(channel_constrained);
  uint8_t channel_address = channelToChannelAddress(channel_constrained);
  long analog_value = voltageToAnalogValue(channel,voltage);
  setAnalogValueOnChip(chip,channel_address,analog_value);
}

void AD57X4R::setAllVoltages(double voltage)
{
  for (size_t channel=0; channel<getChannelCount(); ++channel)
  {
    setVoltage(channel,voltage);
  }
}

double AD57X4R::analogValueToVoltage(size_t channel,
  long analog_value)
{
  double voltage = 0.0;
  size_t channel_constrained = constrain(channel,
    CHANNEL_MIN,
    getChannelCount()-1);
  long analog_value_min = getAnalogValueMin(channel_constrained);
  long analog_value_max = getAnalogValueMax(channel_constrained);
  long analog_value_constrained = constrain(analog_value,
    analog_value_min,
    analog_value_max);
  if ((analog_value_constrained < 0) && rangeIsBipolar(range_[channel_constrained]))
  {
    double voltage_min = getVoltageMin(channel_constrained);
    voltage = (analog_value_constrained*voltage_min)/analog_value_min;
  }
  else
  {
    double voltage_max = getVoltageMax(channel_constrained);
    voltage = (analog_value_constrained*voltage_max)/analog_value_max;
  }
  return voltage;
}

long AD57X4R::voltageToAnalogValue(size_t channel,
  double voltage)
{
  long analog_value = 0;
  size_t channel_constrained = constrain(channel,
    CHANNEL_MIN,
    getChannelCount()-1);
  double voltage_min = getVoltageMin(channel_constrained);
  double voltage_max = getVoltageMax(channel_constrained);
  double voltage_constrained = constrain(voltage,
    voltage_min,
    voltage_max);
  if ((voltage_constrained < 0) && rangeIsBipolar(range_[channel_constrained]))
  {
    long analog_value_min = getAnalogValueMin(channel_constrained);
    analog_value = (voltage_constrained*analog_value_min)/voltage_min;
  }
  else
  {
    long analog_value_max = getAnalogValueMax(channel_constrained);
    analog_value = (voltage_constrained*analog_value_max)/voltage_max;
  }
  return analog_value;
}

bool AD57X4R::channelPoweredUp(size_t channel)
{
  if (channel >= getChannelCount())
  {
    return false;
  }
  uint8_t chip = channelToChip(channel);
  uint8_t channel_address = channelToChannelAddress(channel);
  uint16_t data = readPowerControlRegister(chip);

  bool channel_powered_up = false;
  switch (channel_address)
  {
    case CHANNEL_ADDRESS_A:
      channel_powered_up = data & POWER_CONTROL_DAC_A;
      break;
    case CHANNEL_ADDRESS_B:
      channel_powered_up = data & POWER_CONTROL_DAC_B;
      break;
    case CHANNEL_ADDRESS_C:
      channel_powered_up = data & POWER_CONTROL_DAC_C;
      break;
    case CHANNEL_ADDRESS_D:
      channel_powered_up = data & POWER_CONTROL_DAC_D;
      break;
  }
  return channel_powered_up;
}

bool AD57X4R::referencePoweredUp(uint8_t chip)
{
  if (chip >= chip_count_)
  {
    return false;
  }
  uint16_t data = readPowerControlRegister(chip);

  bool reference_powered_up = data & POWER_CONTROL_REF;
  return reference_powered_up;
}

bool AD57X4R::thermalShutdown(uint8_t chip)
{
  if (chip >= chip_count_)
  {
    return false;
  }
  uint16_t data = readPowerControlRegister(chip);

  bool thermal_shutdown = data & POWER_CONTROL_THERMAL_SHUTDOWN;
  return thermal_shutdown;
}

bool AD57X4R::channelOverCurrent(size_t channel)
{
  if (channel >= getChannelCount())
  {
    return false;
  }
  uint8_t chip = channelToChip(channel);
  uint8_t channel_address = channelToChannelAddress(channel);
  uint16_t data = readPowerControlRegister(chip);

  bool channel_over_current = false;
  switch (channel_address)
  {
    case CHANNEL_ADDRESS_A:
      channel_over_current = data & POWER_CONTROL_OVERCURRENT_A;
      break;
    case CHANNEL_ADDRESS_B:
      channel_over_current = data & POWER_CONTROL_OVERCURRENT_B;
      break;
    case CHANNEL_ADDRESS_C:
      channel_over_current = data & POWER_CONTROL_OVERCURRENT_C;
      break;
    case CHANNEL_ADDRESS_D:
      channel_over_current = data & POWER_CONTROL_OVERCURRENT_D;
      break;
  }
  return channel_over_current;
}

void AD57X4R::beginSimultaneousUpdate()
{
  if (simultaneous_update_enabled_)
  {
    digitalWrite(ldac_pin_,HIGH);
  }
}

void AD57X4R::simultaneousUpdate()
{
  if (simultaneous_update_enabled_)
  {
    digitalWrite(ldac_pin_,LOW);
  }
}

// private
void AD57X4R::initialize()
{
  simultaneous_update_enabled_ = false;
}

uint8_t AD57X4R::channelToChip(size_t channel)
{
  uint8_t chip = channel / CHANNEL_COUNT_PER_CHIP;
  return chip;
}

uint8_t AD57X4R::channelToChannelAddress(size_t channel)
{
  uint8_t chip_channel = channel % CHANNEL_COUNT_PER_CHIP;
  uint8_t channel_address;
  switch (chip_channel)
  {
    case 0:
      channel_address = CHANNEL_ADDRESS_A;
      break;
    case 1:
      channel_address = CHANNEL_ADDRESS_B;
      break;
    case 2:
      channel_address = CHANNEL_ADDRESS_C;
      break;
    case 3:
      channel_address = CHANNEL_ADDRESS_D;
      break;
  }
  return channel_address;
}

void AD57X4R::enableClockSelect()
{
  digitalWrite(chip_select_pin_,LOW);
}

void AD57X4R::disableClockSelect()
{
  digitalWrite(chip_select_pin_,HIGH);
}
void AD57X4R::spiBeginTransaction()
{
  SPI.beginTransaction(SPISettings(SPI_CLOCK,SPI_BIT_ORDER,SPI_MODE));
  enableClockSelect();
}

void AD57X4R::spiEndTransaction()
{
  disableClockSelect();
  SPI.endTransaction();
}

void AD57X4R::initializeMosiDatagramArray(AD57X4R::Datagram datagram_array[])
{
  Datagram mosi_datagram;
  mosi_datagram.bytes = 0;
  mosi_datagram.rw = RW_WRITE;
  mosi_datagram.reg = REGISTER_CONTROL;
  mosi_datagram.channel_address = CONTROL_ADDRESS_NOP;
  for (size_t chip_n=0; chip_n<chip_count_; ++chip_n)
  {
    datagram_array[chip_n] = mosi_datagram;
  }
}

void AD57X4R::writeMosiDatagramToChip(int chip,
  AD57X4R::Datagram mosi_datagram)
{
  Datagram mosi_datagram_array[chip_count_];
  initializeMosiDatagramArray(mosi_datagram_array);
  mosi_datagram_array[chip] = mosi_datagram;

  spiBeginTransaction();
  for (int chip_n=(chip_count_ - 1); chip_n>=0; --chip_n)
  {
    Datagram mosi_datagram_n = mosi_datagram_array[chip];
    for (int byte_n=(DATAGRAM_SIZE - 1); byte_n>=0; --byte_n)
    {
      uint8_t byte_write = (mosi_datagram_n.bytes >> (8*byte_n)) & 0xff;
      SPI.transfer(byte_write);
    }
  }
  spiEndTransaction();
}

AD57X4R::Datagram AD57X4R::readMisoDatagramFromChip(int chip)
{
  Datagram mosi_datagram_array[chip_count_];
  initializeMosiDatagramArray(mosi_datagram_array);

  Datagram miso_datagram_array[chip_count_];

  spiBeginTransaction();
  for (int chip_n=(chip_count_ - 1); chip_n>=0; --chip_n)
  {
    miso_datagram_array[chip].bytes = 0;
    for (int byte_n=(DATAGRAM_SIZE - 1); byte_n>=0; --byte_n)
    {
      uint8_t byte_write = (mosi_datagram_array[chip].bytes >> (8*byte_n)) & 0xff;
      uint8_t byte_read = SPI.transfer(byte_write);
      miso_datagram_array[chip].bytes |= ((uint32_t)byte_read) << (8*byte_n);
    }
  }
  spiEndTransaction();

  Datagram miso_datagram;
  miso_datagram.bytes = 0;
  if ((chip >=0) && (chip < chip_count_))
  {
    miso_datagram = miso_datagram_array[chip];
  }

  return miso_datagram;
}

void AD57X4R::powerUpAllDacs()
{
  uint16_t data = (POWER_CONTROL_DAC_A |
    POWER_CONTROL_DAC_B |
    POWER_CONTROL_DAC_C |
    POWER_CONTROL_DAC_D |
    POWER_CONTROL_REF);

  Datagram mosi_datagram;
  mosi_datagram.bytes = 0;
  mosi_datagram.rw = RW_WRITE;
  mosi_datagram.reg = REGISTER_POWER_CONTROL;
  mosi_datagram.channel_address = CHANNEL_ADDRESS_POWER_CONTROL;
  mosi_datagram.data = data;
  int chip = CHIP_ALL;
  writeMosiDatagramToChip(chip,mosi_datagram);
}

void AD57X4R::setOutputRangeOnChip(int chip,
  uint8_t channel_address,
  Range range)
{
  uint16_t data;
  switch (range)
  {
    case UNIPOLAR_5V:
      data = OUTPUT_RANGE_UNIPOLAR_5V;
      break;
    case UNIPOLAR_10V:
      data = OUTPUT_RANGE_UNIPOLAR_10V;
      break;
    case UNIPOLAR_10V8:
      data = OUTPUT_RANGE_UNIPOLAR_10V8;
      break;
    case BIPOLAR_5V:
      data = OUTPUT_RANGE_BIPOLAR_5V;
      break;
    case BIPOLAR_10V:
      data = OUTPUT_RANGE_BIPOLAR_10V;
      break;
    case BIPOLAR_10V8:
      data = OUTPUT_RANGE_BIPOLAR_10V8;
      break;
    default:
      data = OUTPUT_RANGE_UNIPOLAR_5V;
      break;
  }
  Datagram mosi_datagram;
  mosi_datagram.bytes = 0;
  mosi_datagram.rw = RW_WRITE;
  mosi_datagram.reg = REGISTER_OUTPUT_RANGE;
  mosi_datagram.channel_address = channel_address;
  mosi_datagram.data = data;
  writeMosiDatagramToChip(chip,mosi_datagram);
}

void AD57X4R::setAnalogValueOnChip(int chip,
  uint8_t channel_address,
  long data)
{
  Datagram mosi_datagram;
  mosi_datagram.bytes = 0;
  mosi_datagram.rw = RW_WRITE;
  mosi_datagram.reg = REGISTER_DAC;
  mosi_datagram.channel_address = channel_address;
  switch (resolution_)
  {
    case AD5754R:
      mosi_datagram.data = data;
      break;
    case AD5734R:
      mosi_datagram.data = data << 2;
      break;
    case AD5724R:
      mosi_datagram.data = data << 4;
      break;
  }
  writeMosiDatagramToChip(chip,mosi_datagram);
}

void AD57X4R::load(int chip)
{
  Datagram mosi_datagram;
  mosi_datagram.bytes = 0;
  mosi_datagram.rw = RW_WRITE;
  mosi_datagram.reg = REGISTER_CONTROL;
  mosi_datagram.channel_address = CONTROL_ADDRESS_LOAD;
  writeMosiDatagramToChip(chip,mosi_datagram);
}

uint16_t AD57X4R::readPowerControlRegister(uint8_t chip)
{
  Datagram mosi_datagram;
  mosi_datagram.bytes = 0;
  mosi_datagram.rw = RW_READ;
  mosi_datagram.reg = REGISTER_POWER_CONTROL;
  mosi_datagram.channel_address = CHANNEL_ADDRESS_POWER_CONTROL;
  writeMosiDatagramToChip(chip,mosi_datagram);

  Datagram miso_datagram = readMisoDatagramFromChip(chip);
  return miso_datagram.data;
}

bool AD57X4R::rangeIsBipolar(AD57X4R::Range range)
{
  bool range_is_bipolar = false;
  if ((range == BIPOLAR_5V) || (range == BIPOLAR_10V) || (range == BIPOLAR_10V8))
  {
    range_is_bipolar = true;
  }
  return range_is_bipolar;
}
