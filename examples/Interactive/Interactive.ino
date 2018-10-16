#include <AD57X4R.h>
#include <Streaming.h>


const size_t CHIP_SELECT_PIN = 10;
const size_t LOAD_DAC_PIN = 3;
const size_t CLEAR_PIN = 4;
const long BAUD = 115200;

const size_t CHANNEL_COUNT = 4;
const double VOLTAGE_MIN = -10.8;
const double VOLTAGE_MAX = 10.8;
const long ANALOG_MIN = -32768;
const long ANALOG_MAX = 65536;

AD57X4R dac = AD57X4R(CHIP_SELECT_PIN);

char input_buffer[128];
uint8_t idx = 0;
bool input_complete = false;
char *argv[8];
int arg1, arg2, arg3;
const size_t CHANNEL_MIN = 0;
const size_t CHANNEL_MAX = CHANNEL_COUNT-1;

uint8_t parse(char *line, char **argv, uint8_t max_args)
{
  uint8_t arg_count = 0;
  while (*line != '\0')
  { /* if not the end of line ....... */
    while (*line == ',' || *line == ' ' || *line == '\t' || *line == '\n')
    {
      *line++ = '\0'; /* replace commas and white spaces with 0 */
    }
    *argv++ = line; /* save the argument position */
    arg_count++;
    if (arg_count == max_args-1)
    {
      break;
    }
    while (*line != '\0' && *line != ',' && *line != ' ' &&
      *line != '\t' && *line != '\n')
    {
      line++; /* skip the argument until ... */
    }
  }
  *argv = '\0'; /* mark the end of argument list */
  return arg_count;
}

void setup()
{
  // PC communications
  Serial.begin(BAUD);

  // Initialize DAC
  dac.setLoadDacPin(LOAD_DAC_PIN);
  dac.setClearPin(CLEAR_PIN);
  dac.setup(AD57X4R::AD5754R);
  size_t channel = 0;
  dac.setOutputRange(channel,AD57X4R::BIPOLAR_10V);
  channel = 1;
  dac.setOutputRange(channel,AD57X4R::UNIPOLAR_10V);
  channel = 2;
  dac.setOutputRange(channel,AD57X4R::UNIPOLAR_5V);
  channel = 3;
  dac.setOutputRange(channel,AD57X4R::BIPOLAR_10V8);
}

void loop()
{
  if (input_complete)
  {
    uint8_t arg_count = parse((char*)input_buffer, argv, sizeof(argv));
    if (strcmp(argv[0], "setVoltage") == 0)
    {
      if ((arg_count == 3) && (0 < strlen(argv[1])) && (0 < strlen(argv[2])))
      {
        size_t channel = atoi(argv[1]);
        double voltage = atof(argv[2]);
        dac.setVoltage(channel,voltage);
        long analog_value = dac.voltageToAnalogValue(channel,voltage);
        Serial << "setVoltage CHANNEL: " << channel << ", VOLTAGE: " << voltage << ", ANALOG_VALUE: " << analog_value << "\n";
      }
      else
      {
        Serial << "setVoltage <CHANNEL> <VOLTAGE>, CHANNEL = {" << CHANNEL_MIN << ".." << CHANNEL_MAX << "}" << ", VOLTAGE = {" << VOLTAGE_MIN << ".." << VOLTAGE_MAX << "}" << endl;
      }
    }
    else if (strcmp(argv[0], "setAnalogValue") == 0)
    {
      if ((arg_count == 3) && (0 < strlen(argv[1])) && (0 < strlen(argv[2])))
      {
        size_t channel = atoi(argv[1]);
        long analog_value = atoi(argv[2]);
        dac.setAnalogValue(channel,analog_value);
        double voltage = dac.analogValueToVoltage(channel,analog_value);
        Serial << "setAnalogValue CHANNEL: " << channel << ", ANALOG_VALUE: " << analog_value << ", VOLTAGE: " << voltage << "\n";
      }
      else
      {
        Serial << "setAnalogValue <CHANNEL> <ANALOG_VALUE>, CHANNEL = {" << CHANNEL_MIN << ".." << CHANNEL_MAX << "}" << ", ANALOG_VALUE = {" << ANALOG_MIN << ".." << ANALOG_MAX << "}" << endl;
      }
    }
    else if (strcmp(argv[0], "channelPoweredUp") == 0)
    {
      if (0 < strlen(argv[1]))
      {
        size_t channel = atoi(argv[1]);
        channel = constrain(channel,CHANNEL_MIN,CHANNEL_MAX);
        bool channel_powered_up = dac.channelPoweredUp(channel);
        Serial << "channel " << channel << " powered up = " << channel_powered_up << endl;
      }
      else
      {
        Serial << "channelPoweredUp <CHANNEL>, CHANNEL = {" << CHANNEL_MIN << ".." << CHANNEL_MAX << "}" << endl;
      }
    }
    else
    {
      Serial.println("setVoltage <CHANNEL> <VOLTAGE>, setAnalogValue <CHANNEL> <ANALOG_VALUE>, channelPoweredUp <CHANNEL>");
    }

    input_complete = false;
  }
}

void serialEvent()
{
  while (Serial.available())
  {
    uint8_t in_byte;
    in_byte = Serial.read();
    if ((in_byte == '\n') || (in_byte == '\r'))
    {
      input_buffer[idx] = 0;
      idx = 0;
      input_complete = true;
    }
    else if (((in_byte == '\b') || (in_byte == 0x7f)) && (idx > 0))
    {
      idx--;
    }
    else if ((in_byte >= ' ') && (idx < sizeof(input_buffer) - 1))
    {
      input_buffer[idx++] = in_byte;
    }
  }
}
