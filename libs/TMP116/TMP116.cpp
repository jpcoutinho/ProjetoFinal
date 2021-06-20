#include "TMP116.h"


TMP116::TMP116(const nrfx_twi_t* p_m_twi, uint8_t i2cAddr) 
{
	_addr = i2cAddr; 
  	_m_twi = p_m_twi;
}

double TMP116::readTempC()
{
	return readTemperature();
}

double TMP116::readTemperature()
{
	return read16(TMP116_REG_TEMP) * 0.0078125;
}


uint16_t TMP116::readDeviceId()
{
	return read16(TMP116_REG_DEVICE_ID);
}


/*********************************************************************/

uint16_t TMP116::read16(uint8_t a) {
  
  nrfx_err_t err_code;

  uint8_t buffer[2]; //recebe os 16 bits de temperatura

  err_code = nrfx_twi_tx(_m_twi, _addr, &a, sizeof(a), true); //no stop = true
  //APP_ERROR_CHECK(err_code);  
  err_code = nrfx_twi_rx(_m_twi, _addr, buffer, sizeof(buffer));
  //APP_ERROR_CHECK(err_code);

  uint16_t ret = (buffer[0] << 8) | buffer[1] ;
 
  return ret;
}


void TMP116::write16(uint8_t a, uint16_t v) {
  
  nrfx_err_t err_code;
  uint8_t pec;
  uint8_t pecbuf[4];

  pecbuf[0] = _addr << 1;
  pecbuf[1] = a;
  pecbuf[2] = v & 0xff; //Como v tem 16 bits,
  pecbuf[3] = v >> 0x08;   //essas 2 linhas divide v em 2 partes

  uint8_t reg[] = {a, pecbuf[2], pecbuf[3], pec}; //envia ao endereco do registrador os lsb de v, depois os msb e depois o crc
  err_code = nrfx_twi_tx(_m_twi, _addr, reg, sizeof(reg), false); 
  //APP_ERROR_CHECK(err_code);
}



