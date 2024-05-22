#include "gt911_i2c.h"
#include "stm32h7xx_hal.h"


#define GTP_INT_GPIO_PORT   GPIOA
#define GTP_INT_GPIO_PIN    GPIO_PIN_7
#define GTP_RST_GPIO_PORT   GPIOC
#define GTP_RST_GPIO_PIN    GPIO_PIN_4


#define TP_PRES_DOWN 0x80  //����������	  
#define TP_CATH_PRES 0x40  //�а��������� 
#define CT_MAX_TOUCH  5    //������֧�ֵĵ���,�̶�Ϊ5��

#define  SETBIT_SCL(x)             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, x)//SCL
#define  SETBIT_SDA(x)             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, x)//SDA
#define  CT_READ_SDA               HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)
#define  GT_RST(x)                 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, x)//pc4 rst
I2C_HandleTypeDef hi2c3;//i2c���
#define		GT911_I2C			hi2c3

// ����IC�豸��ַ
#define GT911_DIV_W 0x28
#define GT911_DIV_R 0x29

//IO��������	 

#define GT_INT    		PCin(15)		//GT911�ж�����	    PC4
   	
 
//I2C��д����	
#define GT_CMD_WR 		0X28    //д����
#define GT_CMD_RD 		0X29		//������
 

#define GT_TOUCH_MAX	5			//����gt911�����ͬʱ��ȡ5�������������
//GT911 ���ּĴ�������
#define GT_CTRL_REG 	0X8040   	//GT911���ƼĴ���
#define GT_CFGS_REG 	0X8047   	//GT911������ʼ��ַ�Ĵ���
#define GT_CHECK_REG 	0X80FF   	//GT911У��ͼĴ���
#define GT_PID_REG 		0X8140   	//GT911��ƷID�Ĵ���

#define GT_GSTID_REG 	0X814E   	//GT911��ǰ��⵽�Ĵ������,��7λ�Ǵ�����־λ����4λ�Ǵ�����������

#define GT_TPD_Sta		0X8150		//��������ʼ���ݵ�ַ
#define GT_TP1_REG 		0X8150  	//��һ�����������ݵ�ַ
#define GT_TP2_REG 		0X8158		//�ڶ������������ݵ�ַ
#define GT_TP3_REG 		0X8160		//���������������ݵ�ַ
#define GT_TP4_REG 		0X8168		//���ĸ����������ݵ�ַ
#define GT_TP5_REG 		0X8170		//��������������ݵ�ַ


typedef enum
{
	X_L = 0,
	X_H = 1,
	Y_L = 2,
	Y_H = 3,
	S_L	= 4,
	S_H = 5
}Data_XYS_P;	//����X��Y��������С����ƫ����

typedef enum
{
	TOUCH__NO		= 0x00,	//û�д���
	TOUCH_ING		= 0x80	//������
}TOUCH_STATE_enum;	//����״̬

typedef struct
{
	uint16_t	X_Point;	//X����
	uint16_t	Y_Point;	//Y����
	uint16_t	S_Point;	//�������С
}XY_Coordinate;	//����������


/*�����ṹ��*/
typedef struct
{
	uint8_t Touch_State				;	//����״̬
	uint8_t Touch_Number			;	//��������
	XY_Coordinate Touch_XY[GT_TOUCH_MAX]	;	//������x���꣬����gt911���5���������
}Touch_Struct;	//������Ϣ�ṹ��

Touch_Struct	User_Touch;

void CT_SDA_OUT(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;	
	
	 
  GPIO_InitStruct.Pin = GPIO_PIN_6; //pa4 scl pa6 sda
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void CT_SDA_IN(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;	
	
	 
  GPIO_InitStruct.Pin = GPIO_PIN_6; //pa4 scl pa6 sda
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
void CT_IIC_Init(void)
{	
  GPIO_InitTypeDef  GPIO_InitStruct;	
	
	 
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6; //pa4 sckl pa6 sda
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4|GPIO_PIN_6,1);
	
//	GPIOC->CRL&=0XFFFF0FF0;	//PC0,PC3 �������
//	GPIOC->CRL|=0X00003003;	   
//	GPIOC->ODR|=1<<1;	    //PC0 �����	 
//	GPIOC->ODR|=1<<3;	    //PC3 �����	 
}



void CT_IIC_Start(void)
{
	CT_SDA_OUT();
	SETBIT_SDA(1);	  	  
	SETBIT_SCL(1);
	CT_Delay();
 	SETBIT_SDA(0);//START:when CLK is high,DATA change form high to low 
	CT_Delay();
	SETBIT_SCL(0);//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  

void CT_IIC_Stop(void)
{ 
CT_SDA_OUT();
	SETBIT_SDA(0);	  	  
	SETBIT_SCL(0);
	CT_Delay();
 	SETBIT_SDA(1);//START:when CLK is high,DATA change form high to low 
	CT_Delay();
	SETBIT_SCL(1);
    CT_Delay();
}



uint8_t CT_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	CT_SDA_IN();      //SDA����Ϊ����  
	SETBIT_SDA(1);CT_Delay();	   
	SETBIT_SCL(1);CT_Delay();	 
	while(CT_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			CT_IIC_Stop();
			return 1;
		} 
	}
	SETBIT_SCL(0);//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void CT_IIC_Ack(void)
{
	SETBIT_SCL(0);
	CT_SDA_OUT();
	SETBIT_SDA(0);
	CT_Delay();
	SETBIT_SCL(1);
	CT_Delay();
	SETBIT_SCL(0);
}
//������ACKӦ��		    
void CT_IIC_NAck(void)
{
	SETBIT_SCL(0);
	CT_SDA_OUT();
	SETBIT_SDA(1);
	CT_Delay();
	SETBIT_SCL(1);
	CT_Delay();
	SETBIT_SCL(0);
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void CT_IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	CT_SDA_OUT(); 	    
    SETBIT_SCL(0);//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        SETBIT_SDA((txd&0x80)>>7);
        txd<<=1; 	      
		SETBIT_SCL(1);
		CT_Delay();
		SETBIT_SCL(0);	
		CT_Delay();
    }	 
} 	 

uint8_t CT_IIC_Read_Byte(unsigned char ack)
{
	uint8_t i,receive=0;
 	CT_SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        SETBIT_SCL(0); 	    	   
		CT_Delay();
		SETBIT_SCL(1);  
		receive<<=1;
		if(CT_READ_SDA)
        {receive++;   
        }
	}	  				 
	if (!ack)CT_IIC_NAck();//����nACK
	else CT_IIC_Ack(); //����ACK   
 	return receive;
}

uint8_t GT911_WR_Reg(uint16_t reg,uint8_t *buf,uint8_t len)
{
	uint8_t i;
	uint8_t ret=0;
	CT_IIC_Start();	
 	CT_IIC_Send_Byte(GT_CMD_WR);   	//����д���� 	 
	CT_IIC_Wait_Ack();
	CT_IIC_Send_Byte(reg>>8);   	//���͸�8λ��ַ
	CT_IIC_Wait_Ack(); 	 										  		   
	CT_IIC_Send_Byte(reg&0XFF);   	//���͵�8λ��ַ
	CT_IIC_Wait_Ack();  
	for(i=0;i<len;i++)
	{	   
    	CT_IIC_Send_Byte(buf[i]);  	//������
		ret=CT_IIC_Wait_Ack();
		if(ret)break;  
	}
    CT_IIC_Stop();					//����һ��ֹͣ����	    
	return ret; 
}
//��GT911����һ������
//reg:��ʼ�Ĵ�����ַ
//buf:���ݻ�������
//len:�����ݳ���			  
void GT911_RD_Reg(uint16_t reg,uint8_t *buf,uint8_t len)
{
	uint8_t i; 
 	CT_IIC_Start();	
 	CT_IIC_Send_Byte(GT_CMD_WR);   //����д���� 	 
	CT_IIC_Wait_Ack();
 	CT_IIC_Send_Byte(reg>>8);   	//���͸�8λ��ַ
	CT_IIC_Wait_Ack(); 	 										  		   
 	CT_IIC_Send_Byte(reg&0XFF);   	//���͵�8λ��ַ
	CT_IIC_Wait_Ack();  
 	CT_IIC_Start();  	 	   
	CT_IIC_Send_Byte(GT_CMD_RD);   //���Ͷ�����		   
	CT_IIC_Wait_Ack();	
    
	for(i=0;i<len;i++)
	{	   
    	buf[i]=CT_IIC_Read_Byte(i==(len-1)?0:1); //������	  
	} 
    CT_IIC_Stop();//����һ��ֹͣ����    
} 
//��ʼ��GT911������
//����ֵ:0,��ʼ���ɹ�;1,��ʼ��ʧ�� 



uint8_t GT911_Init(void)
{
    
    
    /*
    TP_PEN    PA7    CT_INT   15
    SPI1_CS   PC4    CT_RST   14
    */
	uint8_t temp[5]={0}; 
	GPIO_InitTypeDef  GPIO_InitStructure;	

	GPIO_InitStructure.Pin =GPIO_PIN_7;// pa7  int
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;	//����
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);  //int����
    
    
	GPIO_InitStructure.Pin =GPIO_PIN_4;// pc4 rst
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;	//�������
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);  //rst����
	

 	CT_IIC_Init();  //��ʼ����������I2C����  
	GT_RST(0);				//��λ
	 HAL_Delay(10);
 	GT_RST(1);				//�ͷŸ�λ 
 	 HAL_Delay(10);
	
GPIO_InitStructure.Pin =GPIO_PIN_4;// 15�˿�����  rst
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;	//�������
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

 	 HAL_Delay(10);
	GT911_RD_Reg(GT_PID_REG,temp,4);//��ȡ��ƷID
	temp[4]=0;

	if(strcmp((char*)temp,"911"))//ID==911
	{
		temp[0]=0X02;			
		GT911_WR_Reg(GT_CTRL_REG,temp,1);//��λGT911
		GT911_RD_Reg(GT_CFGS_REG,temp,1);//��ȡGT_CFGS_REG�Ĵ���
//		if(temp[0]<0X60)//Ĭ�ϰ汾�Ƚϵ�,��Ҫ����flash����
//		{
//			printf("Default Ver:%d\r\n",temp[0]);
//		}
		CT_Delay();
		temp[0]=0X00;	 
		GT911_WR_Reg(GT_CTRL_REG,temp,1);//������λ   
		return 0;
	} 
    
	
	return 1;
    
}




// GT911�������ã�Ĭ��ʹ�õ�һ�������ļ��������Ļ�쳣���ɳ���ʹ�õڶ���
#if  1
const uint8_t CTP_CFG_GT911[] =  {
0x41,0x20,0x03,0xE0,0x01,0x05,0x3D,0x00,0x01,0x08,
0x1E,0x05,0x3C,0x3C,0x03,0x05,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x1A,0x1C,0x1E,0x14,0x8A,0x2A,0x0C,
0x2A,0x28,0xEB,0x04,0x00,0x00,0x01,0x61,0x03,0x2C,
0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x14,0x3C,0x94,0xC5,0x02,0x08,0x00,0x00,0x04,
0xB7,0x16,0x00,0x9F,0x1B,0x00,0x8B,0x22,0x00,0x7B,
0x2B,0x00,0x70,0x36,0x00,0x70,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x18,0x16,0x14,0x12,0x10,0x0E,0x0C,0x0A,
0x08,0x06,0x04,0x02,0xFF,0xFF,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x24,0x22,0x21,0x20,0x1F,0x1E,0x1D,0x1C,
0x18,0x16,0x13,0x12,0x10,0x0F,0x0A,0x08,0x06,0x04,
0x02,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x0A,0x00
};
#else
const uint8_t CTP_CFG_GT911[] =  {
  0x00,0x20,0x03,0xE0,0x01,0x05,0x0D,0x00,0x01,0x08,
  0x28,0x0F,0x50,0x32,0x03,0x05,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x8A,0x2A,0x0C,
  0x45,0x47,0x0C,0x08,0x00,0x00,0x00,0x02,0x02,0x2D,
  0x00,0x00,0x00,0x00,0x00,0x03,0x64,0x32,0x00,0x00,
  0x00,0x28,0x64,0x94,0xC5,0x02,0x07,0x00,0x00,0x04,
  0x9C,0x2C,0x00,0x8F,0x34,0x00,0x84,0x3F,0x00,0x7C,
  0x4C,0x00,0x77,0x5B,0x00,0x77,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x18,0x16,0x14,0x12,0x10,0x0E,0x0C,0x0A,
  0x08,0x06,0x04,0x02,0xFF,0xFF,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x16,0x18,0x1C,0x1D,0x1E,0x1F,0x20,0x21,
  0x22,0x24,0x13,0x12,0x10,0x0F,0x0A,0x08,0x06,0x04,
  0x02,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x24,0x01
};
#endif
void GTP_Init(void)
{
   /*��ʼ��gt9157���豸��ַΪ0x28/0x29*/

   HAL_GPIO_WritePin (GTP_INT_GPIO_PORT,GTP_INT_GPIO_PIN,GPIO_PIN_RESET);
   HAL_GPIO_WritePin (GTP_RST_GPIO_PORT,GTP_RST_GPIO_PIN,GPIO_PIN_RESET);
   HAL_Delay(100);
   /*��λΪ�͵�ƽ��Ϊ��ʼ����׼��*/
   HAL_GPIO_WritePin (GTP_INT_GPIO_PORT,GTP_INT_GPIO_PIN,GPIO_PIN_SET);
   HAL_Delay(10);

   /*����һ��ʱ�䣬���г�ʼ��*/
   HAL_GPIO_WritePin (GTP_RST_GPIO_PORT,GTP_RST_GPIO_PIN,GPIO_PIN_SET);
    HAL_Delay(10);

   /*��INT��������Ϊ��������ģʽ���Ա���մ����ж��ź�*/
   GPIO_InitTypeDef GPIO_InitStructure;
   GPIO_InitStructure.Pin = GTP_INT_GPIO_PIN;
   GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
   GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
   GPIO_InitStructure.Pull  = GPIO_NOPULL;
   HAL_GPIO_Init(GTP_INT_GPIO_PORT, &GPIO_InitStructure);
   
   HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 2);/* �����ж����ȼ� */
   HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);/* ʹ���ж� */
HAL_GPIO_WritePin (GTP_RST_GPIO_PORT,GTP_RST_GPIO_PIN,GPIO_PIN_RESET);//����
HAL_GPIO_WritePin (GTP_INT_GPIO_PORT,GTP_INT_GPIO_PIN,GPIO_PIN_RESET);//����

    HAL_Delay(100);
   uint8_t GTP_ID[4];
   GT911_RD_Reg(GT_PID_REG,GTP_ID,4);
  

   // д�����ñ�
   GT911_WR_Reg(GT_CFGS_REG, (uint8_t *)CTP_CFG_GT911, sizeof(CTP_CFG_GT911));
   HAL_Delay(100);

   // ת��Ϊ��λģʽ
   uint8_t t_temp=2;	//�м����
   GT911_WR_Reg(GT_CTRL_REG, &t_temp, 1);
   HAL_Delay(100);

   // ת��Ϊ��ȡ����ģʽ
   t_temp=0;	//�м����
   GT911_WR_Reg(GT_CTRL_REG, &t_temp, 1);

}



/*
	���ܣ�gt911����ɨ�裬�жϵ�ǰ�Ƿ񱻴���
	����1��
*/
void GTXXXX_Scanf(void)
{
	uint8_t t_temp;	//�м����

	GT911_RD_Reg(GT_GSTID_REG, &t_temp, 1);//��ȡ״̬�Ĵ���

	// ��¼����״̬
	User_Touch.Touch_State = t_temp;
	User_Touch.Touch_Number = (User_Touch.Touch_State & 0x0f);	//��ȡ��������
	User_Touch.Touch_State = (User_Touch.Touch_State & 0x80);	//����״̬

	//�ж��Ƿ��д�������
	switch(User_Touch.Touch_State)
	{
		case TOUCH__NO:		//û������
			break;
		case TOUCH_ING:		//������~�������ݣ�����������
			for(uint8_t i=0; i<User_Touch.Touch_Number; i++)
			{
				GT911_RD_Reg((GT_TPD_Sta + i*8 + X_L), &t_temp, 1);	//��������x����ĵ�8λ
				User_Touch.Touch_XY[i].X_Point  = t_temp;
				GT911_RD_Reg((GT_TPD_Sta + i*8 + X_H), &t_temp, 1);	//��������x����ĸ�8λ
				User_Touch.Touch_XY[i].X_Point |= (t_temp<<8);

				GT911_RD_Reg((GT_TPD_Sta + i*8 + Y_L), &t_temp, 1);	//��������y����ĵ�8λ
				User_Touch.Touch_XY[i].Y_Point  = t_temp;
				GT911_RD_Reg((GT_TPD_Sta + i*8 + Y_H), &t_temp, 1);	//��������y����ĸ�8λ
				User_Touch.Touch_XY[i].Y_Point |= (t_temp<<8);

				GT911_RD_Reg((GT_TPD_Sta + i*8 + S_L), &t_temp, 1);	//����������С���ݵĵ�8λ
				User_Touch.Touch_XY[i].S_Point  = t_temp;
				GT911_RD_Reg((GT_TPD_Sta + i*8 + S_H), &t_temp, 1);	//����������С���ݵĸ�8λ
				User_Touch.Touch_XY[i].S_Point |= (t_temp<<8);
			}

			t_temp=0;
			GT911_WR_Reg(GT_GSTID_REG, &t_temp, 1);	//������ݱ�־λ
		break;
	}
}

void GTP911_Test(void)
{
	GTXXXX_Scanf();		//����ɨ��
	if(User_Touch.Touch_State == 0x80)
	{
		for(uint8_t i=0; i<User_Touch.Touch_Number; i++)
		{
			printf("X : %d  ", User_Touch.Touch_XY[i].X_Point);
			printf("Y : %d  ", User_Touch.Touch_XY[i].Y_Point);
			printf("S : %d\Vr\n\r\n", User_Touch.Touch_XY[i].S_Point);
		}
		User_Touch.Touch_State  = 0;
		User_Touch.Touch_Number = 0;
	}
}

