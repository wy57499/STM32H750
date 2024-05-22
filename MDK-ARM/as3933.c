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


void write_byte(unsigned char addr,unsigned char byte)//дһ���ֽڵ�AS3933�Ĵ���,addr�Ĵ�����ַ��byteҪд��Ĵ���������
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

void  AS3933_INIT (void)	 	//����AS3933��Ӧ�ļĴ����ο�AS3933�����ֲ��16ҳ-18ҹ
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
		write_byte(0x10, 0x00);	 //����ΪУ����ͨ����г��Ƶ�ʣ�����ͨ��DAT�Ź۲죨ʾ���������߼������ǣ�
		HAL_Delay(10);


		//write_byte(0x00, 0xEE);
		write_byte(0x00, 0x0E);//R0<7> = 1 ??32? 12MS  R0<6> = 0 ?????? DAT ????????? R0<5> = 0 ?/????? R0<4 = 1 ??????
		// write_byte(0x01, 0x3A);
		write_byte(0x01, 0x20);//??????? R1<1> = 0
															
		AS3933_RC();			//�ڲ�����У��
		AS3933_RC();

  	write_byte(0x02, 0x20);	//R2�Ĵ������ã�����Ƶ��23-150Khz�����������޷����ľ�����ֵ
  	write_byte(0X04, 0XB0);	//R4�Ĵ������ã�ON/OFFģʽ��OFFʱ��4mS,�����������27K ��������˥��

  	write_byte(0x05, 0x3A);	//R5�Ĵ������ã��ڶ�������ǰ����0x3A
  	write_byte(0x06, 0xC3);	//R6�Ĵ������ã���һ������ǰ����0xC3
  	write_byte(0x07, 0x2B);	//R7�Ĵ������ã���ʱ����50mS,������12

  	write_byte(0x08, 0x00);	//R8�Ĵ������ã�����Ƶ��95-150kHz
		
		
		AS3933_COMM(0xc0);

	while(0)
	{
		// printf("poweroff\n");
		HAL_Delay(100);
		// power_set_soft_poweroff();
	}

/***********************************************����XYZ3ͨ������ƥ�����ֵ*******************************************/
//  	write_byte(0X11, 0x1f);
// 	    write_byte(0X12, 0x1f);
//  	write_byte(0X13, 0x1f);
/********************************************************************************************************************/
}
