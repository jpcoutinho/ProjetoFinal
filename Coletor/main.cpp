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
#include "TMP116.h"
#include "heartRate.h"
//#include "spo2_algorithm.h"

//#define DEBUG

#define LED 21

/*TWI pins*/
#define SDA_PIN 29
#define SCL_PIN 28

/*TWI instance ID. */
#define TWI_INSTANCE_ID 0

/*ESB pipe */
#define PIPE 0

#define OVERFLOW ((uint32_t)(0xFFFFFFFF / APP_TIMER_CLOCK_FREQ))

#define INTERVALO 60000 //milissegundos

typedef struct
{
  uint16_t uid;
  uint8_t tipo;
  uint8_t SpO2;
  uint8_t FC;
  uint8_t temp;
  uint8_t EC;

} health_data;

static health_data vitalSigns;

static nrf_esb_payload_t tx_payload = NRF_ESB_CREATE_PAYLOAD(PIPE, 0x01);
static nrf_esb_payload_t rx_payload;

//-----------------------------------ESB-------------------------------------------//

void nrf_esb_event_handler(nrf_esb_evt_t const *p_event) {

  switch (p_event->evt_id) {
  case NRF_ESB_EVENT_TX_SUCCESS:
#ifdef DEBUG
    printf("TX SUCCESS EVENT\n");
#endif
    break;
  case NRF_ESB_EVENT_TX_FAILED:
#ifdef DEBUG
    printf("TX FAILED EVENT\n");
#endif
    (void)nrf_esb_flush_tx();
    (void)nrf_esb_start_tx();
    break;
  case NRF_ESB_EVENT_RX_RECEIVED:
#ifdef DEBUG
    printf("RX RECEIVED EVENT\n");
#endif
    while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS) {

      if (rx_payload.length > 0) {
#ifdef DEBUG
        printf("RX RECEIVED ACK PAYLOAD\n");
#endif

        //memcpy_fast(&vitalSigns, rx_payload.data, sizeof(health_data));
      }
    }
    break;
  }
}

void clocks_start(void) {

  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;

  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    ;
}

uint32_t esb_init(void) {

  uint32_t err_code;
  uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
  uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
  uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

  nrf_esb_config_t nrf_esb_config = NRF_ESB_DEFAULT_CONFIG;

  nrf_esb_config.protocol = NRF_ESB_PROTOCOL_ESB_DPL;
  nrf_esb_config.retransmit_delay = 600;
  nrf_esb_config.bitrate = NRF_ESB_BITRATE_250KBPS;
  nrf_esb_config.event_handler = nrf_esb_event_handler;
  nrf_esb_config.mode = NRF_ESB_MODE_PTX;
  nrf_esb_config.selective_auto_ack = false;

  err_code = nrf_esb_init(&nrf_esb_config);
  VERIFY_SUCCESS(err_code);

  err_code = nrf_esb_set_base_address_0(base_addr_0);
  VERIFY_SUCCESS(err_code);

  err_code = nrf_esb_set_base_address_1(base_addr_1);
  VERIFY_SUCCESS(err_code);

  err_code = nrf_esb_set_prefixes(addr_prefix, NRF_ESB_PIPE_COUNT);
  VERIFY_SUCCESS(err_code);

  tx_payload.length = sizeof(health_data);

  vitalSigns.temp = 0;
  vitalSigns.FC = 0;
  vitalSigns.SpO2 = 0;
  vitalSigns.tipo = 1;
  vitalSigns.uid = 666;

  memcpy_fast(tx_payload.data, &vitalSigns, sizeof(health_data));

  return err_code;
}
//---------------------------------------------------------------------------------//

//---------------------------------------Funcoes de timer (millis)-----------------------------------//
uint32_t compareMillis(uint32_t previousMillis, uint32_t currentMillis) {
  if (currentMillis < previousMillis)
    return (currentMillis + OVERFLOW + 1 - previousMillis);
  return (currentMillis - previousMillis);
} //retorno em milissegundos

uint32_t millis(void) {
  return (app_timer_cnt_get() / 32.768);
} //retorno em milissegundos

void repeated_timer_handler(void *p_context) {
  nrf_gpio_pin_toggle(LED);
}

/**@brief Create timer.
 */
static void create_timer_millis(app_timer_id_t timer_id, app_timer_mode_t mode = APP_TIMER_MODE_REPEATED, app_timer_timeout_handler_t callback = repeated_timer_handler) {
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
      .frequency = NRF_TWI_FREQ_100K,
      .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
      .hold_bus_uninit = false};

  err_code = nrfx_twi_init(&m_twi, &twi_config, NULL, NULL);
  nrfx_twi_enable(&m_twi);
}
//------------------------------------------------------------------------------------------------//

//-----------------------------------------LFCLK-----------------------------------------------------//
/** @brief Function starting the internal LFCLK XTAL oscillator.
 */
//Essa funcao  necessaria sempre que nao estivermos usando o softdevice
static void lfclk_config(void) {
  ret_code_t err_code = nrf_drv_clock_init();
  //APP_ERROR_CHECK(err_code);

  nrf_drv_clock_lfclk_request(NULL);
}
//----------------------------------------------------------------------------------------------------//

int main(void) {

  ret_code_t err_code;

  nrf_gpio_cfg_output(LED);
  nrf_gpio_pin_write(LED, 1);

  clocks_start();
  lfclk_config();
  app_timer_init();

  //create_timer();
  APP_TIMER_DEF(timerMillis);
  create_timer_millis(timerMillis);
  app_timer_start(timerMillis, APP_TIMER_TICKS(60000), NULL);

  err_code = esb_init();
  //APP_ERROR_CHECK(err_code);

  MAX30105 particleSensor(&m_twi);
  TMP116 tmp116 = TMP116(&m_twi);
  twi_init();

  if (particleSensor.begin() == false) {
    printf("MAX30105 was not found. Please check wiring/power.\n");
    while (1)
      ;
  }

  particleSensor.setup();                  //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(60); //Turn Red LED to low to indicate sensor is running

#ifdef DEBUG
  printf("Coletor inicializado.\n");
#endif

  uint32_t startTime = 0;

  const uint8_t RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
  uint8_t rates[RATE_SIZE];    //Array of heart r1ates
  uint8_t rateSpot = 0;
  uint32_t lastBeat = millis(); //Time at which the last beat occurred
  float beatsPerMinute;
  uint16_t beatAvg;

  while (true) {

    //Temperatura
    vitalSigns.temp = tmp116.readTempC();

    //Batimento cardiaco
    particleSensor.wakeUp();
    nrf_delay_ms(50);

    startTime = millis();

    while (compareMillis(startTime, millis()) < 10000) { //Mede FC por 5 segundos

      uint32_t irValue = particleSensor.getRed();

      if (irValue > 50000) {
        if (checkForBeat(irValue) == true) {
          uint32_t delta = compareMillis(lastBeat, millis());
          lastBeat = millis();

          beatsPerMinute = 60 / (delta / 1000.0);

          if (beatsPerMinute < 255 && beatsPerMinute > 20) {
            rates[rateSpot++] = (uint8_t)beatsPerMinute;
            rateSpot %= RATE_SIZE;

            beatAvg = 0;

            for (uint8_t x = 0; x < RATE_SIZE; x++) {
              beatAvg += rates[x];
            }

            beatAvg /= RATE_SIZE;
          }
        }

        else {
	#ifdef DEBUG
          printf("No finger?\n");
	#endif
        }
      }
    }

    //Radio
    //atualiza o tx_payload antes de retransmitir
    vitalSigns.FC = beatAvg;

    memcpy_fast(tx_payload.data, &vitalSigns, sizeof(health_data));

#ifdef DEBUG
    printf("Avg BPM = %u, Temperatura: %lf\n", vitalSigns.FC, (float)vitalSigns.temp);
#endif

    if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS) {
    }

    else {
#ifdef DEBUG
      printf("Sending packet failed\n");
#endif
    }

    particleSensor.shutDown();

    nrf_delay_ms(INTERVALO);
  }
}
