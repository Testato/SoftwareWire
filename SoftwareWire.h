// SoftwareWire.h
//
// 2008, Raul wrote a I2C with bit banging as an exercise.
// http://codinglab.blogspot.nl/2008/10/i2c-on-avr-using-bit-banging.html
//
// 2010-2012, Tod E. Kurt takes some tricks from Raul,
// and wrote the SoftI2CMaster library for the Arduino environment.
// https://github.com/todbot/SoftI2CMaster
// http://todbot.com/blog/
//
// 2014-2015, Testato updates the SoftI2CMaster library to make it faster
// and to make it compatible with the Arduino 1.x API
// Also changed I2C waveform and added speed selection.
//
// 2015, Peter_n renames the library into "SoftwareWire",
// and made it a drop-in replacement for the Wire library.
//


#ifndef SoftwareWire_h
#define SoftwareWire_h

#include <Arduino.h>


// Transmission status error, the return value of endTransmission()
#define SOFTWAREWIRE_NO_ERROR       0
#define SOFTWAREWIRE_BUFFER_FULL    1
#define SOFTWAREWIRE_ADDRESS_NACK   2
#define SOFTWAREWIRE_DATA_NACK      3
#define SOFTWAREWIRE_OTHER          4

#define SOFTWAREWIRE_BUFSIZE 32        // same as buffer size of Arduino Wire library


class SoftwareWire
{
public:
  SoftwareWire(uint8_t sdaPin, uint8_t sclPin, boolean pullups = true, boolean detectClockStretch = true);
  ~SoftwareWire();
  void end();
  
  void begin();
  void begin(uint8_t address);
  void begin(int address);
  void setClock(uint32_t clock);
  void beginTransmission(uint8_t address);
  void beginTransmission(int address);
  uint8_t endTransmission(boolean sendStop = true);
  uint8_t requestFrom(uint8_t address, uint8_t size, boolean sendStop = true);
  uint8_t requestFrom(int address, int size, boolean sendStop = true);
  uint8_t write(uint8_t data);
  uint8_t write(const uint8_t*, uint8_t size);
  uint8_t write(char* data);
  int available(void);
  int read(void);
  int readBytes(uint8_t* buf, uint8_t size);
  int readBytes(char * buf, uint8_t size);
  int readBytes(char * buf, int size);
  int peek(void);
  void setTimeout(long timeout);           // timeout to wait for the I2C bus
  void printStatus(HardwareSerial& Ser);   // print information to a Serial port

private:
  // per object data

  uint8_t _sdaPin;
  uint8_t _sclPin;
  uint8_t _sdaBitMask;
  uint8_t _sclBitMask;
  
  volatile uint8_t *_sdaPortReg;
  volatile uint8_t *_sclPortReg;
  volatile uint8_t *_sdaDirReg;
  volatile uint8_t *_sclDirReg;
  volatile uint8_t *_sdaPinReg;
  volatile uint8_t *_sclPinReg;

  uint8_t _transmission;      // transmission status, returned by endTransmission(). 0 is no error.
  uint16_t _i2cdelay;         // delay in micro seconds for sda and scl bits.
  boolean _pullups;           // using the internal pullups or not
  boolean _stretch;           // should code handle clock stretching by the slave or not.
  long _timeout;              // timeout in ms. When waiting for a clock pulse stretch.

  uint8_t rxBuf[SOFTWAREWIRE_BUFSIZE];   // buffer inside this class, a buffer per SoftwareWire.
  uint8_t rxBufPut;           // index to rxBuf, just after the last valid byte.
  uint8_t rxBufGet;           // index to rxBuf, the first new to be read byte.
  
  // private methods
  
  void i2c_writebit( uint8_t c );
  uint8_t i2c_readbit(void);
  void i2c_init(void);
  boolean i2c_start(void);
  void i2c_repstart(void);
  void i2c_stop(void);
  uint8_t i2c_write(uint8_t c);
  uint8_t i2c_read(boolean ack);
};

#endif // SoftwareWire_h

