#ifndef I2C_WRAPPER
#define I2C_WRAPPER

#include <Wire.h>

#define SUCCESS  0
#define ADDR_ERR 1
#define DATA_ERR 2

static uint8_t i2cset(uint8_t address, uint8_t *data, uint8_t len, bool stp)
{
  //address = (address<<1);
  Wire.beginTransmission(address);
  for(int i = 0; i < len; i++) { Wire.write(data[i]); }
  // This needs to return 0
  return Wire.endTransmission(stp);
}

static void i2cget(uint8_t address, uint8_t reg, uint8_t *data, uint8_t len)
{
  uint8_t tx_data[1] = {reg};
  i2cset(address, tx_data, 1, false);

  Wire.requestFrom(address, len);
  int i = 0;
  while(Wire.available()){
    if(len <= i)
      return;
    data[i] = Wire.read();
  }
}
#endif
