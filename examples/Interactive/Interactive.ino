#include <AD57X4R.h>
#include <Streaming.h>


const size_t CHIP_SELECT_PIN = 10;
const size_t LOAD_DAC_PIN = 3;
const size_t CLEAR_PIN = 4;
const long BAUD = 115200;

const int MILLIVOLT_MIN = -10000;
const int MILLIVOLT_MAX = 10000;

AD57X4R dac = AD57X4R(CHIP_SELECT_PIN);

char input_buffer[128];
uint8_t idx = 0;
bool input_complete = false;
char *argv[8];
int arg1, arg2, arg3;
long dac_value_min;
long dac_value_max;
size_t channel_min = 0;
size_t channel_max;

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
  dac.setOutputRangeAll(AD57X4R::BIPOLAR_10V);

  dac_value_min = dac.getMinDacValue();
  dac_value_max = dac.getMaxDacValue();

  channel_max = dac.getChannelCount() - 1;
}


void loop()
{
  if (input_complete)
  {
    uint8_t arg_count = parse((char*)input_buffer, argv, sizeof(argv));
    if (strcmp(argv[0], "analogWrite") == 0)
    {
      if ((arg_count == 3) && (0 < strlen(argv[1])) && (0 < strlen(argv[2])))
      {
        size_t channel = atoi(argv[1]);
        channel = constrain(channel,channel_min,channel_max);
        int millivolt_value = atoi(argv[2]);
        millivolt_value = constrain(millivolt_value,MILLIVOLT_MIN,MILLIVOLT_MAX);
        long dac_value = map(millivolt_value,MILLIVOLT_MIN,MILLIVOLT_MAX,dac_value_min,dac_value_max);
        dac.analogWrite(channel,dac_value);
        Serial << "analogWrite CHANNEL: " << channel << ", MILLIVOLT_VALUE: " << millivolt_value << ", DAC_VALUE: " << dac_value << "\n";
      }
      else
      {
        Serial << "analogWrite <CHANNEL> <MILLIVOLT_VALUE>, CHANNEL = {" << channel_min << ".." << channel_max << "}" << ", MILLIVOLT_VALUE = {" << MILLIVOLT_MIN << ".." << MILLIVOLT_MAX << "}" << endl;
      }
    }
    else if (strcmp(argv[0], "channelPoweredUp") == 0)
    {
      if (0 < strlen(argv[1]))
      {
        size_t channel = atoi(argv[1]);
        channel = constrain(channel,channel_min,channel_max);
        bool channel_powered_up = dac.channelPoweredUp(channel);
        Serial << "channel " << channel << " powered up = " << channel_powered_up << endl;
      }
      else
      {
        Serial << "channelPoweredUP <CHANNEL>, CHANNEL = {" << channel_min << ".." << channel_max << "}" << endl;
      }
    }
    else
    {
      Serial.println("analogWrite <CHANNEL> <MILLIVOLT_VALUE>, channelPoweredUp <CHANNEL>");
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
