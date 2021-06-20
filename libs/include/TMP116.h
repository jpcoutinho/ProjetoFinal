//https://training.ti.com/node/1136406

#ifndef TMP116_h
#define TMP116_h

#include <stdint.h>
#include "nrfx_twi.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

#define TMP116_I2CADDR 0x48 //1001 000x

enum TMP116_REGISTERS
{
	TMP116_REG_TEMP = 0x00,
	TMP116_REG_DEVICE_ID = 0x0F,
};

class TMP116 {
public:

	TMP116(const nrfx_twi_t* p_m_twi, uint8_t addr = TMP116_I2CADDR);

	void address(uint8_t address);

	double readTempC();

	uint16_t readDeviceId();

private:

	double readTemperature();


  	uint16_t read16(uint8_t addr);
  	void write16(uint8_t addr, uint16_t data);
  	
  	uint8_t _addr;

  	const nrfx_twi_t* _m_twi;
	
};

#endif
