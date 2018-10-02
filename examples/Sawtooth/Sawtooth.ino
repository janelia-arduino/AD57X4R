#include <AD57X4R.h>
#include <Streaming.h>


const int LOOP_DELAY = 10;
const size_t CHIP_SELECT_PIN = 10;
const size_t LOAD_DAC_PIN = 3;
const size_t CLEAR_PIN = 4;
const long BAUD = 115200;

const int VALUE_INC = 1000;

AD57X4R dac = AD57X4R(CHIP_SELECT_PIN);
int power_control_register;
unsigned int value = 0;

void setup()
{
  // Setup serial communications
  Serial.begin(BAUD);
  Serial.println("* System ready *");

  // Initialize DAC
  dac.setLoadDacPin(LOAD_DAC_PIN);
  dac.setClearPin(CLEAR_PIN);
  dac.init(AD57X4R::AD5754R, AD57X4R::UNIPOLAR_5V);

  // Check to make sure power_control_register set properly
  power_control_register = dac.readPowerControlRegister();
  Serial << "power_control_register =  " << _BIN(power_control_register) << endl;
}


void loop()
{
  value += VALUE_INC;
  if (value > dac.getMaxDacValue())
  {
    value = 0;
  }
  dac.analogWrite(AD57X4R::ALL,value);
  delay(LOOP_DELAY);
}
