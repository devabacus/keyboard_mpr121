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
			
    }
}


