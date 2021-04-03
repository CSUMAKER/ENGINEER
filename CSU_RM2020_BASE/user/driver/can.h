#ifndef	_CAN_H
#define	_CAN_H

#include "makos_includes.h"


#define Control_ID				  	0x200//Ö÷¿Ø
#define canflag_ID                 	0x100


void CAN1_Configuration(uint16_t IdHigh,uint16_t IdLow,u8 mode);
void CAN1_GPIO_Configuration(void);
void CAN1_NVIC_Configuration(void);
void CAN1_mode_Configuration(uint16_t IdHigh,uint16_t IdLow,uint8_t mode);

void can2_Configuration(uint16_t IdHigh,uint16_t IdLow,u8 mode);
void can2_GPIO_Configuration(void);
void can2_NVIC_Configuration(void);
void can2_mode_Configuration(uint16_t IdHigh,uint16_t IdLow,uint8_t mode);


void	can_set(CanTxMsg* msg, U32 id, U8* data);
void	can_set_id(CanTxMsg* msg, U32 id);

void CAN1_SendMsg(u16 ID, u8 *data);
void CAN2_SendMsg(u16 ID, u8 *data);




#endif
