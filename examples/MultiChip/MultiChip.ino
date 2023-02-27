#include <AD57X4R.h>


const size_t CHIP_SELECT_PIN = 10;
const size_t LOAD_DAC_PIN = 3;
const size_t CLEAR_PIN = 4;
const size_t CHIP_COUNT = 2;

const size_t VOLTAGE_INC = 1;
const int DELAY = 500;

AD57X4R dac = AD57X4R(CHIP_SELECT_PIN);
long analog_value = 0;

void setup()
{
  // Initialize DAC
  dac.setLoadDacPin(LOAD_DAC_PIN);
  dac.setClearPin(CLEAR_PIN);
  dac.setup(AD57X4R::AD5724R, CHIP_COUNT);

  dac.setAllOutputRanges(AD57X4R::UNIPOLAR_10V);
}

void loop()
{
  dac.beginSimultaneousUpdate();
  for (size_t channel=0; channel<dac.getChannelCount(); ++channel)
  {
    dac.setVoltage(channel, channel*VOLTAGE_INC);
  }
  dac.simultaneousUpdate();

  delay(DELAY);

  dac.beginSimultaneousUpdate();
  dac.setAllVoltages(0);
  dac.simultaneousUpdate();

  delay(DELAY);
}
