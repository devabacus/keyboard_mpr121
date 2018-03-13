#include <stdio.h>
#include "nrf_drv_twi.h"

#include "SEGGER_RTT.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "mpr121.h"


/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

mpr_t mpr;

nrf_drv_twi_config_t twi_config = {
       .scl                = 19,
       .sda                = 20,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
};

void evt_handler(polarity_t polarity, uint8_t pin)
{
	if(polarity == POLARITY_TOUCHED)
		{
			SEGGER_RTT_printf(0, "%d touched \r\n", pin);
		}
	else
		{
			SEGGER_RTT_printf(0, "%d released \r\n", pin);
		}
}

int main(void)
{
		mpr.address = 0x5A;
		mpr.irq_pin = 18;
		mpr.evt_handler = evt_handler;
	
   	mpr_init(m_twi, &twi_config, &mpr);	

    while (true)
    {
			#ifdef DEBUG
				//SEGGER_RTT_printf(0, "%04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d\r\n", readRegister16(0x04), readRegister16(0x06), readRegister16(0x08), readRegister16(0x0A), readRegister16(0x0C), readRegister16(0x0E), readRegister16(0x10), readRegister16(0x12), readRegister16(0x14), readRegister16(0x16), readRegister16(0x18), readRegister16(0x1A));
				SEGGER_RTT_printf(0, " %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d  ", (readRegister8(0x1E) * 4), (readRegister8(0x1F) * 4), (readRegister8(0x20) * 4), (readRegister8(0x21) * 4), (readRegister8(0x22) * 4), (readRegister8(0x23) * 4), (readRegister8(0x24) * 4), (readRegister8(0x25) * 4), (readRegister8(0x26) * 4), (readRegister8(0x27) * 4), (readRegister8(0x28) * 4), (readRegister8(0x29) * 4));
				SEGGER_RTT_printf(0, "%04x  \r\n ", readRegister16(MPR121_TOUCHSTATUS_L));
			#endif
    }
}


