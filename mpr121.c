#include <stdio.h>
#include "nrf_drv_twi.h"

#include "SEGGER_RTT.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "mpr121.h"


mpr_t * _p_mpr;

void writeRegister(uint8_t reg, uint8_t value)
{
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
	APP_ERROR_CHECK(nrf_drv_twi_tx(&_twi, _p_mpr->address, data, sizeof(data), false));
}


uint8_t readRegister8(uint8_t reg)  //читаем один регистр
{
	uint8_t rx_data;
	APP_ERROR_CHECK(nrf_drv_twi_tx(&_twi, _p_mpr->address, &reg, sizeof(reg), true));
	APP_ERROR_CHECK(nrf_drv_twi_rx(&_twi, _p_mpr->address, &rx_data, sizeof(rx_data)));
	return rx_data;
}

uint16_t readRegister16(uint8_t reg) //читаем два регистра
{
	uint16_t rx_data;
	uint8_t rx_data1[2];
	nrf_drv_twi_tx(&_twi, _p_mpr->address, &reg, sizeof(reg), true);
	nrf_drv_twi_rx(&_twi, _p_mpr->address, rx_data1, sizeof(rx_data1));
	rx_data	= rx_data1[0];
	rx_data |= (rx_data1[1] << 8);
	return rx_data;
}

void touched(void)  //читаем сотояние электродов и записываем их в переменную
{
  uint16_t t = readRegister16(MPR121_TOUCHSTATUS_L);
  t = t & 0x0FFF;
	_p_mpr->currtouched = t;
}

void twi_init (void)
{
    ret_code_t err_code;

    err_code = nrf_drv_twi_init(&_twi, _twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&_twi);
}

void in_data_handler (nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)  //хенедлер по irq пину MPR121
{
	  touched();
		polarity_t polarity;	
			
			for (uint8_t i=0; i<12; i++) {
				// it if *is* touched and *wasnt* touched before, alert!
				if ((_p_mpr->currtouched & (1 << i)) && !(_p_mpr->lasttouched & (1 << i))) {
					polarity = POLARITY_TOUCHED;
					_p_mpr->evt_handler(polarity, i);			//вызываем хендлер из мэйна		
				}
				// if it *was* touched and now *isnt*, alert!
				if (!(_p_mpr->currtouched & (1 << i)) && (_p_mpr->lasttouched & (1 << i))) {
					polarity = POLARITY_RELEASED;
					_p_mpr->evt_handler(polarity, i);
				}
			}
			// reset our state
			_p_mpr->lasttouched = _p_mpr->currtouched;
}


void mpr_init (nrf_drv_twi_t 			 	p_instance, 	  \
							 nrf_drv_twi_config_t * twi_config, 	\
							 mpr_t										*	p_mpr)
{
	
		_p_mpr = p_mpr;
		_twi_config = twi_config;
		_twi = p_instance;
	
		twi_init();
			
		writeRegister(MPR121_SOFTRESET, 0x63);
								
		uint8_t c = readRegister8(MPR121_CONFIG2);
  
		if (c != 0x24)
			{
				SEGGER_RTT_printf(0, "ERROR");
			}
		
			
			
	/*  For the details on how to set these registers, refer to application	note AN3891 */		
			
												// \R/ Rising			
		writeRegister(MPR121_MHDR, 0x01); 
		//Maximum Half Delta (MHD): Determines the largest magnitude of variation to pass through the baseline filter. 
		//The range of the effective value is 1~63. 
			
		writeRegister(MPR121_NHDR, 0x01);
		//Noise Half Delta (NHD): Determines the incremental change when non-noise drift is detected. 
		//The range of the effective value is 1~63
			
		writeRegister(MPR121_NCLR, 0x0E);
		//Noise Count Limit (NCL): Determines the number of samples consecutively greater than the Max Half Delta necessary
		//before it can be determined that it is non-noise. The range of the effective value is 0~255
			
		writeRegister(MPR121_FDLR, 0x00);
		//Filter Delay Count Limit (FDL): Determines the rate of operation of the filter. A larger number makes it operate slower. 
		//	The range of the effective value is 0~255.
			
												// \F/ Falling 			
		writeRegister(MPR121_MHDF, 0x01);
		writeRegister(MPR121_NHDF, 0x05);
		writeRegister(MPR121_NCLF, 0xA0);
		writeRegister(MPR121_FDLF, 0x00);
			
	/*ExTTH, ExRTH: Each of the 13 channels has its own set of touch threshold and release threshold registers. The threshold is
		defined as a deviation value from the baseline value, so it remains constant even baseline value changes. Typically the touch
		threshold is a little bigger than the release threshold to touch debounce and hysteresis.
		The range of the value is 0~255. For the operation details and how to set the threshold refer to 
		application note AN3892 and MPR121 design guidelines. */
			
		writeRegister(MPR121_E0TTH , 0x70);
		writeRegister(MPR121_E0RTH , 0x40);
		writeRegister(MPR121_E1TTH , 0x70);
		writeRegister(MPR121_E1RTH , 0x40);
		writeRegister(MPR121_E2TTH , 0x70);
		writeRegister(MPR121_E2RTH , 0x40);
		writeRegister(MPR121_E3TTH , 0x70);
		writeRegister(MPR121_E3RTH , 0x40);
		writeRegister(MPR121_E4TTH , 0x70);
		writeRegister(MPR121_E4RTH , 0x40);
		writeRegister(MPR121_E5TTH , 0x70);
		writeRegister(MPR121_E5RTH , 0x40);
		writeRegister(MPR121_E6TTH , 0x70);
		writeRegister(MPR121_E6RTH , 0x40);
		writeRegister(MPR121_E7TTH , 0x70);
		writeRegister(MPR121_E7RTH , 0x40);
		writeRegister(MPR121_E8TTH , 0x70);
		writeRegister(MPR121_E8RTH , 0x40);	
		writeRegister(MPR121_E9TTH , 0x70);
		writeRegister(MPR121_E9RTH , 0x40);
		writeRegister(MPR121_E10TTH, 0x70);
		writeRegister(MPR121_E10RTH, 0x40);
		writeRegister(MPR121_E11TTH, 0x70);
		writeRegister(MPR121_E11RTH, 0x40);	
				
		writeRegister(MPR121_DEBOUNCE, 0);
		
		/*  For the details on how to set these registers, refer to application	note AN3890 */	
		writeRegister(MPR121_CONFIG1, 0x10); // default, 16uA charge current
		writeRegister(MPR121_CONFIG2, 0x20); // 0.5uS encoding, 1ms period
			
//	  writeRegister(MPR121_AUTOCONFIG0, 0x8F);

//		writeRegister(MPR121_UPLIMIT, 0x9C);
//		writeRegister(MPR121_TARGETLIMIT, 0x8C);
//		writeRegister(MPR121_LOWLIMIT, 0x65);
		
		#ifdef DEBUG		
		for (uint8_t i=0; i<0x7F; i++) 
			{
				if( readRegister8(i) > 0)
					{
						SEGGER_RTT_printf(0, "%02x =  0x%02x \r\n", i, readRegister8(i));  
						nrf_delay_ms(5);
					}
			}
		#endif
			
		writeRegister(MPR121_ECR, 0x8f); //This register selects measurement electrodes and proximity detection electrode configurations, and controls MPR121 in Run Mode or Stop Mode
						
		nrf_drv_gpiote_init();
		nrf_drv_gpiote_in_config_t in_data_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
		in_data_config.pull = NRF_GPIO_PIN_PULLUP;
		
		nrf_drv_gpiote_in_init (18, &in_data_config, in_data_handler);
		nrf_drv_gpiote_in_event_enable(18, true);
			
		#ifdef DEBUG2	
		while (true)
			{
			//SEGGER_RTT_printf(0, "%04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d\r\n", readRegister16(0x04), readRegister16(0x06), readRegister16(0x08), readRegister16(0x0A), readRegister16(0x0C), readRegister16(0x0E), readRegister16(0x10), readRegister16(0x12), readRegister16(0x14), readRegister16(0x16), readRegister16(0x18), readRegister16(0x1A));
				SEGGER_RTT_printf(0, " %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d  ", (readRegister8(0x1E) * 4), (readRegister8(0x1F) * 4), (readRegister8(0x20) * 4), (readRegister8(0x21) * 4), (readRegister8(0x22) * 4), (readRegister8(0x23) * 4), (readRegister8(0x24) * 4), (readRegister8(0x25) * 4), (readRegister8(0x26) * 4), (readRegister8(0x27) * 4), (readRegister8(0x28) * 4), (readRegister8(0x29) * 4));
				SEGGER_RTT_printf(0, "%04x  \r\n ", readRegister16(MPR121_TOUCHSTATUS_L));
			}
		#endif
}

