#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "app_util.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_error.h"

#include "MAX30105.h"
#include "heartRate.h"

#define LED 21

/*TWI pins*/
#define SDA_PIN 29
#define SCL_PIN 28

/*TWI instance ID. */
#define TWI_INSTANCE_ID 0


#define OVERFLOW ((uint32_t)(0xFFFFFFFF / APP_TIMER_CLOCK_FREQ))

bool calculaFC = true;

void clocks_start( void ) {

    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}

//---------------------------------------Funcoes de timer (millis)-----------------------------------//
uint32_t compareMillis(uint32_t previousMillis, uint32_t currentMillis) {
  if (currentMillis < previousMillis)
    return (currentMillis + OVERFLOW + 1 - previousMillis);
  return (currentMillis - previousMillis);
} //retorno em milissegundos

uint32_t millis(void) {
  return (app_timer_cnt_get() / 32.768);
} //retorno em milissegundos


void single_timer_handler(void *p_context) {
  nrf_gpio_pin_toggle(LED);

  calculaFC = false;
}

/**@brief Create timer.
 */
static void create_timer(app_timer_id_t timer_id, app_timer_mode_t mode = APP_TIMER_MODE_SINGLE_SHOT, app_timer_timeout_handler_t callback = single_timer_handler) {
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

  /* pagina 9 - datasheet
  uint8_t ledBrightness = 60; //Options: 0=Off to 255=50mA
  uint8_t sampleAverage = 4;  //Options: 1, 2, 4, 8, 16, 32
  uint8_t ledMode = 2;        //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  uint8_t sampleRate = 100;   //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  uint16_t pulseWidth = 411;  //Options: 69, 118, 215, 411
  uint16_t adcRange = 4096;   //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  //particleSensor.enableAFULL();                                                                  //Enable the almost full interrupt (default is 32 samples)
  //*/

   ret_code_t err_code;

    nrf_gpio_cfg_output(LED);
    nrf_gpio_pin_write(LED, 1);
    nrf_delay_ms(3000);
    nrf_gpio_pin_write(LED, 0);


    clocks_start();
    lfclk_config();
    app_timer_init(); //inicia o timer semelhante o millis()

    APP_TIMER_DEF(timerFC);
    create_timer(timerFC);
   
    
    MAX30105 particleSensor(&m_twi);
    twi_init();

    
    if (particleSensor.begin() == false) {
      printf("MAX30105 was not found. Please check wiring/power.\n");
      while (1);
    }

    particleSensor.setup();                   //Configure sensor with default settings
    particleSensor.setPulseAmplitudeRed(30);  //Turn Red LED to low to indicate sensor is running


    printf("Teste de frequencia cardiaca iniciado.\n");
 
    uint32_t startTime = 0;

    const uint8_t RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
    uint8_t rates[RATE_SIZE];     //Array of heart r1ates
    uint8_t rateSpot = 0;
    uint32_t lastBeat = millis(); //Time at which the last beat occurred
    float beatsPerMinute;
    uint16_t beatAvg;

    app_timer_start(timerFC, APP_TIMER_TICKS(20000), NULL);
    
    while (true) {

  
        particleSensor.wakeUp();
        
        nrf_delay_ms(50);

  
        //Batimento cardiaco
        
        while (calculaFC) { //Mede FC por 20 segundos

          uint32_t irValue = particleSensor.getRed();

          if (irValue > 50000) {
            if (checkForBeat(irValue) == true) {
              //We sensed a beat!
              uint32_t delta = compareMillis(lastBeat, millis()); //tem que usar a compare millis para verificacao pq ela considera overflow do timer
              lastBeat = millis();

              beatsPerMinute = 60 / (delta / 1000.0);

              if (beatsPerMinute < 255 && beatsPerMinute > 20) {
                rates[rateSpot++] = (uint8_t)beatsPerMinute; //Store this reading in the array
                rateSpot %= RATE_SIZE;                       //Wrap variable

                //Take average of readings
                beatAvg = 0;

                for (uint8_t x = 0; x < RATE_SIZE; x++) {
                  beatAvg += rates[x];
                }

                beatAvg /= RATE_SIZE;
              }
            }
             
            printf("IR = %lu, BPM = %lf, Avg BPM = %u\n", irValue, beatsPerMinute, beatAvg);
          }

          else {
              printf("No finger?\n");
          }
        }
        

        particleSensor.shutDown();

        nrf_delay_ms(2000);
    }
}