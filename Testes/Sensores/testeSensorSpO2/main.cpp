/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "app_error.h"
#include "app_timer.h"
#include "app_util.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_error.h"
#include "nrf_esb.h"
#include "nrf_esb_error_codes.h"
#include "nrf_gpio.h"
#include "sdk_common.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "MAX30105.h"
#include "spo2_algorithm.h"

#define LED 21

/*TWI pins*/
#define SDA_PIN 29
#define SCL_PIN 28

/*TWI instance ID. */
#define TWI_INSTANCE_ID 0

#define OVERFLOW ((uint32_t)(0xFFFFFFFF / APP_TIMER_CLOCK_FREQ))

void clocks_start(void) {

  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;

  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    ;
}

//---------------------------------------Funcoes de timer (millis)-----------------------------------//

/*TRANSFORMAR ESSE BLOCO TODO NUM MODULO GENERICO*/
uint32_t compareMillis(uint32_t previousMillis, uint32_t currentMillis) {
  if (currentMillis < previousMillis)
    return (currentMillis + OVERFLOW + 1 - previousMillis);
  return (currentMillis - previousMillis);
} //retorno em milissegundos

uint32_t millis(void) {
  return (app_timer_cnt_get() / 32.768);
} //retorno em milissegundos

void repeated_timer_handler(void *p_context) {
  
}

/**@brief Create timer.
 */
static void create_timer(app_timer_id_t timer_id, app_timer_mode_t mode = APP_TIMER_MODE_REPEATED, app_timer_timeout_handler_t callback = repeated_timer_handler) {
  ret_code_t err_code;
  // Create timers
  err_code = app_timer_create(&timer_id, mode, callback);
  APP_ERROR_CHECK(err_code);
}

//------------------------------------------------------------------------------------------------//

//-----------------------------------------TWI---------------------------------------------------//
/* TWI instance. */
static const nrfx_twi_t m_twi = NRFX_TWI_INSTANCE(TWI_INSTANCE_ID); //macro que cria uma instacia master do twi

/**
 * @brief TWI initialization.
 */
void twi_init(void) {
  nrfx_err_t err_code;

  const nrfx_twi_config_t twi_config = {
      .scl = SCL_PIN,
      .sda = SDA_PIN,
      .frequency = NRF_TWI_FREQ_100K, //limitado em 100k porque e a maior velocidade suportada pelo termometro IR
      .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
      .hold_bus_uninit = false //liga o pullup das portas apos desabilitar o twi (unit). (para casos onde pode faltar alimentacao no pullup do barramento).
  };

  err_code = nrfx_twi_init(&m_twi, &twi_config, NULL, NULL);
  nrfx_twi_enable(&m_twi);
}
//------------------------------------------------------------------------------------------------//

//-----------------------------------------LFCLK-----------------------------------------------------//
/** @brief Function starting the internal LFCLK XTAL oscillator.
 */
//Essa funcao e necessaria sempre que nao estivermos usando o softdevice
static void lfclk_config(void) {
  ret_code_t err_code = nrf_drv_clock_init();
  APP_ERROR_CHECK(err_code);

  nrf_drv_clock_lfclk_request(NULL);
}
//----------------------------------------------------------------------------------------------------//

int main(void) {

  ret_code_t err_code;

  nrf_gpio_cfg_output(LED);
  nrf_gpio_pin_write(LED, 1);

  clocks_start();
  lfclk_config();
  app_timer_init(); //inicia o modulo timer

  APP_TIMER_DEF(timerSpO2);
  create_timer(timerSpO2);
  app_timer_start(timerSpO2, APP_TIMER_TICKS(20000), NULL);

  MAX30105 particleSensor(&m_twi);
  twi_init();

  if (particleSensor.begin() == false) {
    printf("MAX30105 was not found. Please check wiring/power.\n");
    while (1)
      ;
  }

  uint8_t ledBrightness = 255; //Options: 0=Off to 255=50mA
  uint8_t sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  uint8_t ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  uint8_t sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings


  uint32_t irBuffer[100]; //infrared LED sensor data
  uint32_t redBuffer[100];  //red LED sensor data
  

  int32_t bufferLength; //data length
  int32_t spo2; //SPO2 value
  int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
  int32_t heartRate; //heart rate value
  int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

  printf("Teste de SpO2 iniciado.\n");


  while (true) {

    //particleSensor.wakeUp();

    nrf_delay_ms(50);

    //SpO2
    bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

    //read the first 100 samples, and determine the signal range
    for (uint8_t i = 0; i < bufferLength; i++) {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check();                   //Check the sensor for new data

      redBuffer[i] = particleSensor.getIR();
      irBuffer[i] = particleSensor.getRed();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
    }

    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
    while (1) {
      //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
      for (uint8_t i = 25; i < 100; i++) {
        redBuffer[i - 25] = redBuffer[i];
        irBuffer[i - 25] = irBuffer[i];
      }

      //take 25 sets of samples before calculating the heart rate.
      for (uint8_t i = 75; i < 100; i++) {
        while (particleSensor.available() == false) //do we have new data?
          particleSensor.check();                   //Check the sensor for new data

        nrf_gpio_pin_toggle(LED);

        redBuffer[i] = particleSensor.getIR();
        irBuffer[i] = particleSensor.getRed();
        particleSensor.nextSample(); //We're finished with this sample so move to next sample

   
        printf("FC = %ld", heartRate);
        printf(", FCvalido = %u", validHeartRate);
        printf(", SPO2 = %ld", spo2);
        printf(", SPO2Valido\n = %u", validSPO2);
      }

      //After gathering 25 new samples recalculate HR and SP02
      maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    }

    //particleSensor.shutDown();
  }
}