#ifndef MPR121_H__
#define MPR121_H__

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

//#define DEBUG

typedef enum
{
  POLARITY_TOUCHED,    ///<  Touched
  POLARITY_RELEASED    ///<  Released.
} polarity_t;

static nrf_drv_twi_t _twi;
static nrf_drv_twi_config_t * _twi_config;

typedef	void (*p_in_data_hendler_t)(polarity_t, uint8_t);


typedef struct
{
	uint8_t address;
	uint16_t lasttouched;
	uint16_t currtouched;
	uint8_t irq_pin;
	p_in_data_hendler_t	 evt_handler;
	
} mpr_t;


void mpr_init (nrf_drv_twi_t 			 	p_instance, 	  \
							 nrf_drv_twi_config_t * twi_config, 	\
							 mpr_t										*	p_mpr);

#endif
