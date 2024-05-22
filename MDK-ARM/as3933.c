#include "as3933.h"
#include "main.h"

#define CS_125K_GPIO_Port           GPIOB
#define CS_125K_Pin                 GPIO_PIN_12


#define CS_AS3933_GPIO_clk_Port     GPIOB
#define CS_AS3933_clk_Pin              GPIO_PIN_13
/*HARDWARE*/


#define CS_AS3933_H()    HAL_GPIO_WritePin(CS_125K_GPIO_Port, CS_125K_Pin,GPIO_PIN_SET)
#define CS_AS3933_L()  HAL_GPIO_WritePin(CS_125K_GPIO_Port, CS_125K_Pin,GPIO_PIN_RESET)


extern SPI_HandleTypeDef hspi2;

unsigned char as3933_buf[16] = {0};

void AS3933_COMM(unsigned char com)
{
	HAL_GPIO_WritePin(CS_125K_GPIO_Port, CS_125K_Pin, GPIO_PIN_SET);

	HAL_SPI_Transmit(&hspi2, &com, 1, 10000);
	
	HAL_GPIO_WritePin(CS_125K_GPIO_Port, CS_125K_Pin, GPIO_PIN_RESET);	
}
unsigned char read_byte(unsigned char addr)
{
	unsigned char  temp;
	HAL_GPIO_WritePin(CS_125K_GPIO_Port, CS_125K_Pin, GPIO_PIN_SET);
	temp = addr | 0x40;
	HAL_SPI_Transmit(&hspi2, &temp, 1, 10000);
	HAL_SPI_Receive(&hspi2, &temp, 1, 10000); 
	HAL_GPIO_WritePin(CS_125K_GPIO_Port, CS_125K_Pin, GPIO_PIN_RESET);
	return temp;
}


void write_byte(unsigned char addr,unsigned char byte)//写一个字节到AS3933寄存器,addr寄存器地址，byte要写入寄存器的数据
{
	HAL_GPIO_WritePin(CS_125K_GPIO_Port, CS_125K_Pin, GPIO_PIN_SET);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 10000);
	HAL_SPI_Transmit(&hspi2, &byte, 1, 10000); 
	
	HAL_GPIO_WritePin(CS_125K_GPIO_Port, CS_125K_Pin, GPIO_PIN_RESET);
}

void AS3933_RC(void)
{
	unsigned char addr = 0xC2;
	
	HAL_GPIO_WritePin(CS_125K_GPIO_Port, CS_125K_Pin, GPIO_PIN_SET);
	
	HAL_SPI_Transmit(&hspi2, &addr, 1, 10000);
	
	HAL_SPI_Transmit_DMA(&hspi2,as3933_buf, 8); 
	
	HAL_GPIO_WritePin(CS_125K_GPIO_Port, CS_125K_Pin, GPIO_PIN_RESET);	
}

void  AS3933_INIT (void)	 	//配置AS3933相应的寄存器参考AS3933数据手册第16页-18夜
{
	
	unsigned char temp;

    HAL_Delay(30);

		AS3933_COMM(0xc0);	
    HAL_Delay(30);
		write_byte(0x11, 0x12);  //180p
		HAL_Delay(10);
		write_byte(0x10, 0x41);	 //R16
    HAL_Delay(30);
    write_byte(0x10, 0x00);
    HAL_Delay(30);
		temp = read_byte(0x11);
		if(temp == 0x12)
		{
            for(uint8_t i=0;i<5;i++)
			{
                HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_0);
                 HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_1);
                HAL_Delay(100);
            }
		}

		write_byte(0x12, 0x12);  //120p+10p
		HAL_Delay(10);
		write_byte(0x10, 0x42);	 //R16	
		HAL_Delay(30);
    write_byte(0x10, 0x00);
    HAL_Delay(30);

		write_byte(0x13, 0x15);  //120P
		HAL_Delay(10);
		write_byte(0x10, 0x44);  //
		HAL_Delay(30);
		write_byte(0x10, 0x00);	 //以上为校正各通道的谐振频率，可以通过DAT脚观察（示波器或者逻辑分析仪）
		HAL_Delay(10);


		//write_byte(0x00, 0xEE);
		write_byte(0x00, 0x0E);//R0<7> = 1 ??32? 12MS  R0<6> = 0 ?????? DAT ????????? R0<5> = 0 ?/????? R0<4 = 1 ??????
		// write_byte(0x01, 0x3A);
		write_byte(0x01, 0x20);//??????? R1<1> = 0
															
		AS3933_RC();			//内部晶振校验
		AS3933_RC();

  	write_byte(0x02, 0x20);	//R2寄存器设置：接收频率23-150Khz，降低数据限幅器的绝对阈值
  	write_byte(0X04, 0XB0);	//R4寄存器设置：ON/OFF模式下OFF时长4mS,天线阻尼电阻27K ，无增益衰减

  	write_byte(0x05, 0x3A);	//R5寄存器设置：第二个唤醒前导码0x3A
  	write_byte(0x06, 0xC3);	//R6寄存器设置：第一个唤醒前导码0xC3
  	write_byte(0x07, 0x2B);	//R7寄存器设置：超时设置50mS,波特率12

  	write_byte(0x08, 0x00);	//R8寄存器设置：唤醒频率95-150kHz
		
		
		AS3933_COMM(0xc0);

	while(0)
	{
		// printf("poweroff\n");
		HAL_Delay(100);
		// power_set_soft_poweroff();
	}

/***********************************************设置XYZ3通道天线匹配电容值*******************************************/
//  	write_byte(0X11, 0x1f);
// 	    write_byte(0X12, 0x1f);
//  	write_byte(0X13, 0x1f);
/********************************************************************************************************************/
}
