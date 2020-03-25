#!/usr/bin/env python
'''
these tables are generated from the STM32 datasheets for the
STM32F40x
'''

from utils_dict import *

# additional build information for ChibiOS
build = {
    "CHIBIOS_STARTUP_MK"  : "os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk",
    "CHIBIOS_PLATFORM_MK" : "os/hal/ports/STM32/STM32F4xx/platform.mk"
    }

# MCU parameters
mcu = {
    # location of MCU serial number
    'UDID_START' : 0x1FFF7A10,

    # ram map, as list of (address, size-kb, flags)
    # flags of 1 means DMA-capable
    # flags of 2 means faster memory for CPU intensive work
    'RAM_MAP' : [
        (0x20000000, 128, 1), # main memory, DMA safe
        (0x10000000,  64, 2), # CCM memory, faster, but not DMA safe
    ]
}

DMA_Map = {
	# format is (DMA_TABLE, StreamNum, Channel)
	# extracted from tabula-STM32F4x7-dma.csv
	"ADC1"    	:	[(2,0,0),(2,4,0)],
	"ADC2"    	:	[(2,2,1),(2,3,1)],
	"ADC3"    	:	[(2,0,2),(2,1,2)],
	"CRYP_IN" 	:	[(2,6,2)],
	"CRYP_OUT"	:	[(2,5,2)],
	"DAC1"    	:	[(1,5,7)],
	"DAC2"    	:	[(1,6,7)],
	"DCMI"    	:	[(2,1,1),(2,7,1)],
	"HASH_IN" 	:	[(2,7,2)],
	"I2C1_RX" 	:	[(1,0,1),(1,5,1)],
	"I2C1_TX" 	:	[(1,6,1),(1,7,1)],
	"I2C2_RX" 	:	[(1,2,7),(1,3,7)],
	"I2C2_TX" 	:	[(1,7,7)],
	"I2C3_RX" 	:	[(1,2,3)],
	"I2C3_TX" 	:	[(1,4,3)],
	"I2S2_EXT_RX"	:	[(1,3,3)],
	"I2S2_EXT_TX"	:	[(1,4,2)],
	"I2S3_EXT_RX"	:	[(1,2,2),(1,0,3)],
	"I2S3_EXT_TX"	:	[(1,5,2)],
	"SDIO"    	:	[(2,3,4),(2,6,4)],
	"SPI1_RX" 	:	[(2,0,3),(2,2,3)],
	"SPI1_TX" 	:	[(2,3,3),(2,5,3)],
	"SPI2_RX" 	:	[(1,3,0)],
	"SPI2_TX" 	:	[(1,4,0)],
	"SPI3_RX" 	:	[(1,0,0),(1,2,0)],
	"SPI3_TX" 	:	[(1,5,0),(1,7,0)],
	"TIM1_CH1"	:	[(2,6,0),(2,1,6),(2,3,6)],
	"TIM1_CH2"	:	[(2,6,0),(2,2,6)],
	"TIM1_CH3"	:	[(2,6,0),(2,6,6)],
	"TIM1_CH4"	:	[(2,4,6)],
	"TIM1_COM"	:	[(2,4,6)],
	"TIM1_TRIG"	:	[(2,0,6),(2,4,6)],
	"TIM1_UP" 	:	[(2,5,6)],
	"TIM2_CH1"	:	[(1,5,3)],
	"TIM2_CH2"	:	[(1,6,3)],
	"TIM2_CH3"	:	[(1,1,3)],
	"TIM2_CH4"	:	[(1,6,3),(1,7,3)],
	"TIM2_UP" 	:	[(1,1,3),(1,7,3)],
	"TIM3_CH1"	:	[(1,4,5)],
	"TIM3_CH2"	:	[(1,5,5)],
	"TIM3_CH3"	:	[(1,7,5)],
	"TIM3_CH4"	:	[(1,2,5)],
	"TIM3_TRIG"	:	[(1,4,5)],
	"TIM3_UP" 	:	[(1,2,5)],
	"TIM4_CH1"	:	[(1,0,2)],
	"TIM4_CH2"	:	[(1,3,2)],
	"TIM4_CH3"	:	[(1,7,2)],
	"TIM4_UP" 	:	[(1,6,2)],
	"TIM5_CH1"	:	[(1,2,6)],
	"TIM5_CH2"	:	[(1,4,6)],
	"TIM5_CH3"	:	[(1,0,6)],
	"TIM5_CH4"	:	[(1,1,6),(1,3,6)],
	"TIM5_TRIG"	:	[(1,1,6),(1,3,6)],
	"TIM5_UP" 	:	[(1,0,6),(1,6,6)],
	"TIM6_UP" 	:	[(1,1,7)],
	"TIM7_UP" 	:	[(1,2,1),(1,4,1)],
	"TIM8_CH1"	:	[(2,2,0),(2,2,7)],
	"TIM8_CH2"	:	[(2,2,0),(2,3,7)],
	"TIM8_CH3"	:	[(2,2,0),(2,4,7)],
	"TIM8_CH4"	:	[(2,7,7)],
	"TIM8_COM"	:	[(2,7,7)],
	"TIM8_TRIG"	:	[(2,7,7)],
	"TIM8_UP" 	:	[(2,1,7)],
	"UART4_RX"	:	[(1,2,4)],
	"UART4_TX"	:	[(1,4,4)],
	"UART5_RX"	:	[(1,0,4)],
	"UART5_TX"	:	[(1,7,4)],
	"USART1_RX"	:	[(2,2,4),(2,5,4)],
	"USART1_TX"	:	[(2,7,4)],
	"USART2_RX"	:	[(1,5,4)],
	"USART2_TX"	:	[(1,6,4)],
	"USART3_RX"	:	[(1,1,4)],
	"USART3_TX"	:	[(1,3,4),(1,4,7)],
	"USART6_RX"	:	[(2,1,5),(2,2,5)],
	"USART6_TX"	:	[(2,6,5),(2,7,5)],
}


