/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
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
/** @file
 * @defgroup tw_scanner main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"



#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "SEGGER_RTT.h"
#include "nrf_delay.h"



#define MPR121_I2CADDR_DEFAULT 0x5A

#define MPR121_TOUCHSTATUS_L 0x00
#define MPR121_TOUCHSTATUS_H 0x01
#define MPR121_FILTDATA_0L  0x04
#define MPR121_FILTDATA_0H  0x05
#define MPR121_BASELINE_0   0x1E
#define MPR121_MHDR         0x2B
#define MPR121_NHDR         0x2C
#define MPR121_NCLR         0x2D
#define MPR121_FDLR         0x2E
#define MPR121_MHDF         0x2F
#define MPR121_NHDF         0x30
#define MPR121_NCLF         0x31
#define MPR121_FDLF         0x32
#define MPR121_NHDT         0x33
#define MPR121_NCLT         0x34
#define MPR121_FDLT         0x35

#define MPR121_E0TTH 	 0x41
#define MPR121_E0RTH   0x42
#define MPR121_E1TTH   0x43
#define MPR121_E1RTH   0x44
#define MPR121_E2TTH   0x45
#define MPR121_E2RTH   0x46
#define MPR121_E3TTH   0x47
#define MPR121_E3RTH   0x48
#define MPR121_E4TTH   0x49
#define MPR121_E4RTH   0x4A
#define MPR121_E5TTH   0x4B
#define MPR121_E5RTH   0x4C
#define MPR121_E6TTH   0x4D
#define MPR121_E6RTH   0x4E
#define MPR121_E7TTH   0x4F
#define MPR121_E7RTH   0x50
#define MPR121_E8TTH   0x51
#define MPR121_E8RTH   0x52
#define MPR121_E9TTH   0x53
#define MPR121_E9RTH   0x54
#define MPR121_E10TTH  0x55
#define MPR121_E10RTH  0x56
#define MPR121_E11TTH  0x57
#define MPR121_E11RTH  0x58


#define MPR121_TOUCHTH_0    0x41
#define MPR121_RELEASETH_0    0x42
#define MPR121_DEBOUNCE 0x5B
#define MPR121_CONFIG1 0x5C
#define MPR121_CONFIG2 0x5D
#define MPR121_CHARGECURR_0 0x5F
#define MPR121_CHARGETIME_1 0x6C
#define MPR121_ECR 0x5E
#define MPR121_AUTOCONFIG0 0x7B
#define MPR121_AUTOCONFIG1 0x7C
#define MPR121_UPLIMIT   0x7D
#define MPR121_LOWLIMIT  0x7E
#define MPR121_TARGETLIMIT  0x7F

#define MPR121_GPIODIR  0x76
#define MPR121_GPIOEN  0x77
#define MPR121_GPIOSET  0x78
#define MPR121_GPIOCLR  0x79
#define MPR121_GPIOTOGGLE  0x7A


#define MPR121_SOFTRESET 0x80


/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

 /* Number of possible TWI addresses. */
 #define TWI_ADDRESSES      127

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
uint8_t address = 0x5A;
uint16_t lasttouched = 0;
uint16_t currtouched = 0;


void writeRegister(uint8_t reg, uint8_t value)
{
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
	APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, address, data, sizeof(data), false));
}


uint8_t readRegister8(uint8_t reg)
{
	uint8_t rx_data;
	APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, address, &reg, sizeof(reg), true));
	APP_ERROR_CHECK(nrf_drv_twi_rx(&m_twi, address, &rx_data, sizeof(rx_data)));
	return rx_data;
}

uint16_t readRegister16(uint8_t reg)
{
	uint16_t rx_data;
	uint8_t rx_data1 = 0;
	uint8_t rx_data2 = 0;
	APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, address, &reg, sizeof(reg), true));
	APP_ERROR_CHECK(nrf_drv_twi_rx(&m_twi, address, &rx_data1, sizeof(uint8_t)));
	reg = reg + 1;
	APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, address, &reg, sizeof(reg), true));
	APP_ERROR_CHECK(nrf_drv_twi_rx(&m_twi, address, &rx_data2, sizeof(uint8_t)));
	rx_data	= rx_data1;
	rx_data |= (rx_data2 << 8);
	return rx_data;
}

uint16_t  touched(void) {
  uint16_t t = readRegister16(MPR121_TOUCHSTATUS_L);
  return t & 0x0FFF;
}

/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = 19,
       .sda                = 20,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}


/**
 * @brief Function for main application entry.
 */
int main(void)
{
    ret_code_t err_code;
    
    uint8_t sample_data;
    bool detected_device = false;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("TWI scanner.");
    NRF_LOG_FLUSH();
    twi_init();

    err_code = nrf_drv_twi_rx(&m_twi, address, &sample_data, sizeof(sample_data));
    if (err_code == NRF_SUCCESS)
        {
            detected_device = true;
            NRF_LOG_INFO("TWI device detected at address 0x%x.", address);
        }
        NRF_LOG_FLUSH();
		
		writeRegister(MPR121_SOFTRESET, 0x63);
				

		
		uint8_t c = readRegister8(MPR121_CONFIG2);
  
		if (c != 0x24)
			{
				SEGGER_RTT_printf(0, "ERROR");
			}
				
		writeRegister(MPR121_MHDR, 0x01);
		writeRegister(MPR121_NHDR, 0x01);
		writeRegister(MPR121_NCLR, 0x0E);
		writeRegister(MPR121_FDLR, 0x00);
	
		writeRegister(MPR121_MHDF, 0x01);
		writeRegister(MPR121_NHDF, 0x05);
		writeRegister(MPR121_NCLF, 0xA0);
		writeRegister(MPR121_FDLF, 0x00);
			
		writeRegister(MPR121_E0TTH , 0x70);
		writeRegister(MPR121_E0RTH , 0x40);
		writeRegister(MPR121_E1TTH , 0x50);
		writeRegister(MPR121_E1RTH , 0x40);
		writeRegister(MPR121_E2TTH , 0x50);
		writeRegister(MPR121_E2RTH , 0x40);
		writeRegister(MPR121_E3TTH , 0x50);
		writeRegister(MPR121_E3RTH , 0x40);
		writeRegister(MPR121_E4TTH , 0x50);
		writeRegister(MPR121_E4RTH , 0x40);
		writeRegister(MPR121_E5TTH , 0x50);
		writeRegister(MPR121_E5RTH , 0x40);
		writeRegister(MPR121_E6TTH , 0x50);
		writeRegister(MPR121_E6RTH , 0x40);
		writeRegister(MPR121_E7TTH , 0x50);
		writeRegister(MPR121_E7RTH , 0x40);
		writeRegister(MPR121_E8TTH , 0x50);
		writeRegister(MPR121_E8RTH , 0x40);	
		writeRegister(MPR121_E9TTH , 0x40);
		writeRegister(MPR121_E9RTH , 0x30);
		writeRegister(MPR121_E10TTH, 0x50);
		writeRegister(MPR121_E10RTH, 0x40);
		writeRegister(MPR121_E11TTH, 0x50);
		writeRegister(MPR121_E11RTH, 0x40);	
			
			
				
 
		writeRegister(MPR121_DEBOUNCE, 0);
		writeRegister(MPR121_CONFIG1, 0x10); // default, 16uA charge current
		writeRegister(MPR121_CONFIG2, 0x20); // 0.5uS encoding, 1ms period
			
//	  writeRegister(MPR121_AUTOCONFIG0, 0x8F);

//		writeRegister(MPR121_UPLIMIT, 0x9C);
//		writeRegister(MPR121_TARGETLIMIT, 0x8C);
//		writeRegister(MPR121_LOWLIMIT, 0x65);
			
		for (uint8_t i=0; i<0x7F; i++) 
			{
				if( readRegister8(i) > 0)
					{
						SEGGER_RTT_printf(0, "%02x =  0x%02x \r\n", i, readRegister8(i));  
						nrf_delay_ms(5);
					}
			}
		writeRegister(MPR121_ECR, 0x8f);

    while (true)
    {
			#ifdef DEBUG
			//SEGGER_RTT_printf(0, "%04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d\r\n", readRegister16(0x04), readRegister16(0x06), readRegister16(0x08), readRegister16(0x0A), readRegister16(0x0C), readRegister16(0x0E), readRegister16(0x10), readRegister16(0x12), readRegister16(0x14), readRegister16(0x16), readRegister16(0x18), readRegister16(0x1A));
			SEGGER_RTT_printf(0, " %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d  ", (readRegister8(0x1E) * 4), (readRegister8(0x1F) * 4), (readRegister8(0x20) * 4), (readRegister8(0x21) * 4), (readRegister8(0x22) * 4), (readRegister8(0x23) * 4), (readRegister8(0x24) * 4), (readRegister8(0x25) * 4), (readRegister8(0x26) * 4), (readRegister8(0x27) * 4), (readRegister8(0x28) * 4), (readRegister8(0x29) * 4));
			SEGGER_RTT_printf(0, "%04x  \r\n ", readRegister16(MPR121_TOUCHSTATUS_L));
			#endif
      currtouched = touched();
			
			
			for (uint8_t i=0; i<12; i++) {
				// it if *is* touched and *wasnt* touched before, alert!
				if ((currtouched & (1 << i)) && !(lasttouched & (1 << i))) {
					SEGGER_RTT_printf(0, "%d touched \r\n", i);
				}
				// if it *was* touched and now *isnt*, alert!
				if (!(currtouched & (1 << i)) && (lasttouched & (1 << i))) {
					SEGGER_RTT_printf(0, "%d released \r\n", i);
				}
			}
		
			// reset our state
			lasttouched = currtouched;
			//nrf_delay_ms(100);
    }
}

/** @} */
