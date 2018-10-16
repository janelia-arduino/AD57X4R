#include <AD57X4R.h>


const size_t CHIP_SELECT_PIN = 10;
const size_t LOAD_DAC_PIN = 3;
const size_t CLEAR_PIN = 4;

const long ANALOG_CHANNEL_0 = 0;
const long ANALOG_CHANNEL_1 = 1;

const int LOOP_DELAY = 1000;

AD57X4R dac = AD57X4R(CHIP_SELECT_PIN);
bool is_min_value = true;

void setup()
{
  // Initialize DAC
  dac.setLoadDacPin(LOAD_DAC_PIN);
  dac.setClearPin(CLEAR_PIN);
  dac.setup(AD57X4R::AD5754R);
  dac.setOutputRange(ANALOG_CHANNEL_0,AD57X4R::UNIPOLAR_5V);
  dac.setOutputRange(ANALOG_CHANNEL_1,AD57X4R::BIPOLAR_10V);
}


void loop()
{
  if (is_min_value)
  {
    dac.beginSimultaneousUpdate();
    dac.setAnalogValue(ANALOG_CHANNEL_0,dac.getAnalogValueMin(ANALOG_CHANNEL_0));
    delay(LOOP_DELAY/2);
    dac.setAnalogValue(ANALOG_CHANNEL_1,dac.getAnalogValueMin(ANALOG_CHANNEL_1));
    dac.simultaneousUpdate();
    delay(LOOP_DELAY/2);
  }
  else
  {
    dac.setAnalogValue(ANALOG_CHANNEL_0,dac.getAnalogValueMax(ANALOG_CHANNEL_0));
    delay(LOOP_DELAY/2);
    dac.setAnalogValue(ANALOG_CHANNEL_1,dac.getAnalogValueMax(ANALOG_CHANNEL_1));
    delay(LOOP_DELAY/2);
  }
  is_min_value = !is_min_value;
}
