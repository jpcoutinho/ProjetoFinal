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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "app_util.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_error.h"

#include "TMP116.h"

#define LED 21

/*TWI pins*/
#define SDA_PIN 29
#define SCL_PIN 28

/*TWI instance ID. */
#define TWI_INSTANCE_ID 0

#define OVERFLOW ((uint32_t)(0xFFFFFFFF / APP_TIMER_CLOCK_FREQ))

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

bool imprimir = false;
double temperatura = -1.0;
TMP116 tmp116 = TMP116(&m_twi);;


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



void single_timer_handler(void *p_context) {
  //nrf_gpio_pin_write(LED, 1);
  temperatura = tmp116.readTempC();        
  imprimir = true;
}

void millis_timer_handler(void *p_context) {
}

/**@brief Create timer.
 */
static void create_timer(app_timer_id_t timer_id, app_timer_mode_t mode = APP_TIMER_MODE_SINGLE_SHOT, app_timer_timeout_handler_t callback = single_timer_handler) {
  ret_code_t err_code;
  // Create timers
  err_code = app_timer_create(&timer_id, mode, callback);
  APP_ERROR_CHECK(err_code);
}

static void create_timer_millis(app_timer_id_t timer_id, app_timer_mode_t mode = APP_TIMER_MODE_REPEATED, app_timer_timeout_handler_t callback = millis_timer_handler) {
  ret_code_t err_code;
  // Create timers
  err_code = app_timer_create(&timer_id, mode, callback);
  APP_ERROR_CHECK(err_code);
}
//------------------------------------------------------------------------------------------------//


//-----------------------------------------LFCLK-----------------------------------------------------//
void clocks_start( void ) {

    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}


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
    nrf_delay_ms(3000);
    nrf_gpio_pin_write(LED, 0);


    clocks_start();
    lfclk_config();
    app_timer_init(); //inicia o timer semelhante o millis()

    //create timer;
    APP_TIMER_DEF(timerTemp);
    create_timer(timerTemp);
    app_timer_start(timerTemp, APP_TIMER_TICKS(10000), NULL);

    APP_TIMER_DEF(timerMillis);
    create_timer_millis(timerMillis);
    app_timer_start(timerMillis, APP_TIMER_TICKS(60000), NULL);
    
    twi_init();
  
    printf("Teste de temperatura iniciado\n");

    uint32_t previousMillis = millis();
    

    while (true) {

      if (compareMillis(previousMillis, millis()) > 1000){
        nrf_gpio_pin_toggle(LED);
        previousMillis = millis();
      }
           
        while (imprimir){
          printf("Temperatura lida: %lf*C\n", temperatura);
          nrf_delay_ms(2000);
        }
    }
}