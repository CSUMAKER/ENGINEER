 #ifndef	_headfile_h
#define	_headfile_h

//由于硬件板子io口不一致请选择板子类型：
//MAIN_CONTROL_2019：范金泽的2019主控板
//MAIN_CONTROL_2020solider：黄伟的2020步兵主控
//MAIN_CONTROL_2020sentry：边照龙的2020哨兵主控
/***********三选一，否则会崩*******/

//#define MAIN_CONTROL_2019
//#define MAIN_CONTROL_2020solider
#define MAIN_CONTROL_2020sentry

/*********************************/

#if defined(MAIN_CONTROL_2020solider)||defined(MAIN_CONTROL_2020sentry)
#define USE_NEW_OLED
#endif

//****MAKOS_RTOS****
#include "makos_includes.h"
//****other****
#include "data_fifo.h"
#include "protocol.h"

//****arithmetic****
#include "math.h"
#include "arm_math.h"

//****device****
#include "flash_data.h"
#include "task_remote.h"
#include "timer.h"
#include "rng_led.h"

//****diver****
#include "led.h"
#include "stm32_flash.h"
#include "usart.h"
#include "user_usart.h"
#include "can.h"
#include "user_can.h"
#include "uart_dma.h"
#include "task_Ano.h"
#include "mak_filter.h"
#include "UI_lib.h"

//****ST_lib****
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
#endif
