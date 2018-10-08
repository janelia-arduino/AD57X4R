#include <AD57X4R.h>


const size_t CHIP_SELECT_PIN = 10;
const size_t LOAD_DAC_PIN = 3;
const size_t CLEAR_PIN = 4;

const long DAC_CHANNEL = 0;

const int LOOP_DELAY = 10;
const size_t DAC_VALUE_INC = 1000;

AD57X4R dac = AD57X4R(CHIP_SELECT_PIN);
long dac_value = 0;

void setup()
{
  // Initialize DAC
  dac.setLoadDacPin(LOAD_DAC_PIN);
  dac.setClearPin(CLEAR_PIN);
  dac.setup(AD57X4R::AD5754R);
  dac.setOutputRange(DAC_CHANNEL,AD57X4R::UNIPOLAR_5V);
}

void loop()
{
  dac_value += DAC_VALUE_INC;
  if (dac_value > dac.getMaxDacValue(DAC_CHANNEL))
  {
    dac_value = dac.getMinDacValue(DAC_CHANNEL);
  }
  dac.analogWrite(DAC_CHANNEL,dac_value);
  delay(LOOP_DELAY);
}
