/***************************************************
  This is a library for the MLX90614 Temp Sensor

  Designed specifically to work with the MLX90614 sensors in the
  adafruit shop
  ----> https://www.adafruit.com/products/1748
  ----> https://www.adafruit.com/products/1749

  These sensors use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_MLX90614.h"


/**
 * @brief Construct a new Adafruit_MLX90614::Adafruit_MLX90614 object
 *
 * @param i2caddr The I2C address to use. Defaults to 0x5A
 */
Adafruit_MLX90614::Adafruit_MLX90614(const nrfx_twi_t* p_m_twi, uint8_t i2cAddr) { 
  _addr = i2cAddr; 
  _m_twi = p_m_twi;
}


/**
 * @brief Begin the I2C connection
 *
 * @return bool  Always returns true
 */
bool Adafruit_MLX90614::begin() {
  //Wire.begin(); //Funcao inutil. Eu nao deveria iniciar o master pela biblioteca do slave...
  return true;
}


/**
 * @brief Read the raw value from the emissivity register
 *
 * @return uint16_t The unscaled emissivity value
 */
uint16_t Adafruit_MLX90614::readEmissivityReg(void) {
  return read16(MLX90614_EMISS);
}


/**
 * @brief Write the raw unscaled emissivity value to the emissivity register
 *
 * @param ereg The unscaled emissivity value
 */
void Adafruit_MLX90614::writeEmissivityReg(uint16_t ereg) {
  write16(MLX90614_EMISS, 0); // erase
  nrf_delay_ms(10);
  write16(MLX90614_EMISS, ereg);
  nrf_delay_ms(10);
}
/**
 * @brief Read the emissivity value from the sensor's register and scale
 *
 * @return double The emissivity value, ranging from 0.1 - 1.0
 */
double Adafruit_MLX90614::readEmissivity(void) {
  uint16_t ereg = read16(MLX90614_EMISS);
  return ((double)ereg) / 65535.0;
}
/**
 * @brief Set the emissivity value
 *
 * @param emissivity The emissivity value to use, between 0.1 and 1.0
 */
void Adafruit_MLX90614::writeEmissivity(double emissivity) {
  uint16_t ereg = int(0xffff * emissivity);

  writeEmissivityReg(ereg);
}

/**
 * @brief Get the current temperature of an object in degrees Farenheit
 *
 * @return double The temperature in degrees Farenheit
 */
double Adafruit_MLX90614::readObjectTempF(void) {
  return (readTemp(MLX90614_TOBJ1) * 1.8) + 32;
}
/**
 * @brief Get the current ambient temperature in degrees Farenheit
 *
 * @return double The temperature in degrees Farenheit
 */
double Adafruit_MLX90614::readAmbientTempF(void) {
  return (readTemp(MLX90614_TA) * 1.8) + 32;
}

/**
 * @brief Get the current temperature of an object in degrees Celcius
 *
 * @return double The temperature in degrees Celcius
 */
double Adafruit_MLX90614::readObjectTempC(void) {
  return readTemp(MLX90614_TOBJ1);
}

/**
 * @brief Get the current ambient temperature in degrees Celcius
 *
 * @return double The temperature in degrees Celcius
 */
double Adafruit_MLX90614::readAmbientTempC(void) {
  return readTemp(MLX90614_TA);
}

float Adafruit_MLX90614::readTemp(uint8_t reg) {
  float temp;

  temp = read16(reg);
  temp *= 0.02;
  temp -= 273.15;

  return temp;
}


/*********************************************************************/

uint16_t Adafruit_MLX90614::read16(uint8_t a) {
  
  nrfx_err_t err_code;

  uint8_t buffer[3]; //recebe os 16bits + 8 bits crc

  err_code = nrfx_twi_tx(_m_twi, _addr, &a, sizeof(a), true); //no stop = true
  //APP_ERROR_CHECK(err_code);  
  err_code = nrfx_twi_rx(_m_twi, _addr, buffer, sizeof(buffer));
  //APP_ERROR_CHECK(err_code);

  //uint8_t pec = buffer[2]; //não necessario, mas vou deixar aqui por enquanto

  uint16_t ret = (buffer[1] << 8) | buffer[0] ;
 
  return ret;
}


void Adafruit_MLX90614::write16(uint8_t a, uint16_t v) {
  
  nrfx_err_t err_code;
  uint8_t pec;
  uint8_t pecbuf[4];

  pecbuf[0] = _addr << 1;
  pecbuf[1] = a;
  pecbuf[2] = v & 0xff; //Como v tem 16 bits,
  pecbuf[3] = v >> 0x08;   //essas 2 linhas divide v em 2 partes

  pec = crc8(pecbuf, sizeof(pecbuf));

  uint8_t reg[] = {a, pecbuf[2], pecbuf[3], pec}; //envia ao endereco do registrador os lsb de v, depois os msb e depois o crc
  err_code = nrfx_twi_tx(_m_twi, _addr, reg, sizeof(reg), false); 
  //APP_ERROR_CHECK(err_code);
}



uint8_t Adafruit_MLX90614::crc8(uint8_t *addr, uint8_t len)
// The PEC calculation includes all bits except the START, REPEATED START, STOP,
// ACK, and NACK bits. The PEC is a CRC-8 with polynomial X8+X2+X1+1.
{
  uint8_t crc = 0;

  while (len--) 
  {

    uint8_t inbyte = *addr++;

    for (uint8_t i = 8; i; i--) 
    {
      uint8_t carry = (crc ^ inbyte) & 0x80;
      crc <<= 1;

      if (carry)
        crc ^= 0x7;
      inbyte <<= 1;
    }
  }
  
  return crc;
}

