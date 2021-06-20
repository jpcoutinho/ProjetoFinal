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
#include "nrf_esb.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "app_util.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_error.h"


//#define LED 21

/*ESB pipe */
#define PIPE 0


typedef struct
{                       
    uint32_t a;                                   
    uint16_t b;                                                                
    uint8_t packetNumber; 
} dummy_payload;

static nrf_esb_payload_t        tx_payload = NRF_ESB_CREATE_PAYLOAD(PIPE, 0x01);
static nrf_esb_payload_t        rx_payload;


static dummy_payload data;



void clocks_start( void )
{
    // Start HFCLK and wait for it to start.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}



//-----------------------------------ESB-------------------------------------------//

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event) {

    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            #ifdef DEBUG
            printf("TX SUCCESS EVENT\n");
            #endif
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            #ifdef DEBUG
            printf("TX FAILED EVENT\n");
            #endif
            (void) nrf_esb_flush_tx();
            (void) nrf_esb_start_tx();
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            #ifdef DEBUG
            printf("RX RECEIVED EVENT\n");
            #endif
            while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS) {

               // if (rx_payload.length > 0) {
                    printf("RX RECEIVED ACK PAYLOAD\n");
              
                    //printf("ACK Flag: %u\n", vitalSigns.flag);
                   
                //}
            }
            break;
    }
}

uint32_t esb_init( void ) {

    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.tx_output_power          = NRF_ESB_TX_POWER_4DBM;
    nrf_esb_config.retransmit_delay         = 600;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_250KBPS ; //ou 2MBPS 
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
    nrf_esb_config.selective_auto_ack       = false;

    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, NRF_ESB_PIPE_COUNT);
    VERIFY_SUCCESS(err_code);


    tx_payload.length = sizeof(dummy_payload);

    data.a = 1;
    data.b = 100;

    memcpy_fast(tx_payload.data, &data, sizeof(dummy_payload));

    return err_code;
}
//---------------------------------------------------------------------------------//


int main(void) {

    ret_code_t err_code;

   // nrf_gpio_cfg_output(LED);
   // nrf_gpio_pin_write(LED, 1);
   // nrf_delay_ms(3000);
   // nrf_gpio_pin_write(LED, 0);

    clocks_start();
    err_code = esb_init();
    //APP_ERROR_CHECK(err_code);

    printf("Teste de alcance iniciado.\n");
    
    uint8_t packetNumber = 0;
    uint32_t startTime = 0;


    while (true) {

        data.packetNumber = packetNumber;
        memcpy_fast(tx_payload.data, &data, sizeof(dummy_payload));

        #ifdef DEBUG
        printf("Transmitindo pacote numero %u\n", data.packetNumber);
        #endif
        
 
        if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS) {
            packetNumber++;
        }

        else {
            printf("Sending packet failed\n");
        }

        nrf_delay_ms(5000);
    }
}