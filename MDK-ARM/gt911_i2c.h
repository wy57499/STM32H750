#ifndef  gt911_i2c_h
#define  gt911_i2c_h
#include "main.h"

void CT_SDA_OUT(void);
void CT_SDA_IN(void);


void CT_Delay(void);
void CT_IIC_Init(void);
void CT_IIC_Start(void);
void CT_IIC_Stop(void);
uint8_t CT_IIC_Wait_Ack(void);
void CT_IIC_Ack(void);
void CT_IIC_NAck(void);
void CT_IIC_Send_Byte(uint8_t txd);
uint8_t CT_IIC_Read_Byte(unsigned char ack);

uint8_t GT911_Init(void);
uint8_t GT911_WR_Reg(uint16_t reg,uint8_t *buf,uint8_t len);
void GT911_RD_Reg(uint16_t reg,uint8_t *buf,uint8_t len);

void GTP911_Test(void);
void GTXXXX_Scanf(void);
void GTP_Init(void);
#endif

