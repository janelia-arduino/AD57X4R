// ----------------------------------------------------------------------------
// AD57X4R.h
//
// Provides an SPI based interface to the AD57X4R
// Complete, Quad, 12-/14-/16-Bit, Serial Input,
// Unipolar/Bipolar Voltage Output DACs.
//
// Authors:
// Peter Polidoro polidorop@janelia.hhmi.org
// ----------------------------------------------------------------------------
#ifndef AD57X4R_H
#define AD57X4R_H
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "SPI.h"
#include "Streaming.h"


class AD57X4R
{
public:
  enum resolutions {AD5724R, AD5734R, AD5754R};
  enum output_ranges {UNIPOLAR_5V, UNIPOLAR_10V, BIPOLAR_5V, BIPOLAR_10V};
  enum channels {A, B, C, D, ALL};
  AD57X4R();
  AD57X4R(int cs_pin);
  void init(resolutions resolution=AD5754R, output_ranges output_range=UNIPOLAR_5V, boolean spi_reset=false);
  int readPowerControlRegister();
  void analogWrite(channels channel, unsigned int value);
  void analogWrite(channels channel, int value);
  void analogWrite(int pin, unsigned int value);
  void analogWrite(int pin, int value);
  unsigned int getMaxDacValue();
  void setCSInvert();
  void setCSNormal();
private:
  int resolution_;
  int cs_pin_;
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
  boolean spi_reset_;

  void spiBegin();
  void setupCS(int cs_pin);
  void setHeader(byte value, byte bit_shift, byte bit_count);
  void setReadWrite(byte value);
  void setRegisterSelect(byte value);
  void setDACAddress(channels channel);
  void setNOP();
  void sendOutput();
  int readInput();
  void setPowerControlRegister(channels channel);
  void setOutputRange(output_ranges output_range, channels channel);
  void setData(unsigned int value);
  void setData(int value);
  void csEnable();
  void csDisable();
};
#endif
