#include "gt911_i2c.h"
#include "stm32h7xx_hal.h"


#define GTP_INT_GPIO_PORT   GPIOA
#define GTP_INT_GPIO_PIN    GPIO_PIN_7
#define GTP_RST_GPIO_PORT   GPIOC
#define GTP_RST_GPIO_PIN    GPIO_PIN_4


#define TP_PRES_DOWN 0x80  //触屏被按下	  
#define TP_CATH_PRES 0x40  //有按键按下了 
#define CT_MAX_TOUCH  5    //电容屏支持的点数,固定为5点

#define  SETBIT_SCL(x)             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, x)//SCL
#define  SETBIT_SDA(x)             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, x)//SDA
#define  CT_READ_SDA               HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)
#define  GT_RST(x)                 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, x)//pc4 rst
I2C_HandleTypeDef hi2c3;//i2c句柄
#define		GT911_I2C			hi2c3

// 触摸IC设备地址
#define GT911_DIV_W 0x28
#define GT911_DIV_R 0x29

//IO操作函数	 

#define GT_INT    		PCin(15)		//GT911中断引脚	    PC4
   	
 
//I2C读写命令	
#define GT_CMD_WR 		0X28    //写命令
#define GT_CMD_RD 		0X29		//读命令
 

#define GT_TOUCH_MAX	5			//对于gt911，最多同时获取5个触摸点的数据
//GT911 部分寄存器定义
#define GT_CTRL_REG 	0X8040   	//GT911控制寄存器
#define GT_CFGS_REG 	0X8047   	//GT911配置起始地址寄存器
#define GT_CHECK_REG 	0X80FF   	//GT911校验和寄存器
#define GT_PID_REG 		0X8140   	//GT911产品ID寄存器

#define GT_GSTID_REG 	0X814E   	//GT911当前检测到的触摸情况,第7位是触摸标志位，低4位是触摸点数个数

#define GT_TPD_Sta		0X8150		//触摸点起始数据地址
#define GT_TP1_REG 		0X8150  	//第一个触摸点数据地址
#define GT_TP2_REG 		0X8158		//第二个触摸点数据地址
#define GT_TP3_REG 		0X8160		//第三个触摸点数据地址
#define GT_TP4_REG 		0X8168		//第四个触摸点数据地址
#define GT_TP5_REG 		0X8170		//第五个触摸点数据地址


typedef enum
{
	X_L = 0,
	X_H = 1,
	Y_L = 2,
	Y_H = 3,
	S_L	= 4,
	S_H = 5
}Data_XYS_P;	//数据X、Y、触摸大小数据偏移量

typedef enum
{
	TOUCH__NO		= 0x00,	//没有触摸
	TOUCH_ING		= 0x80	//被触摸
}TOUCH_STATE_enum;	//触摸状态

typedef struct
{
	uint16_t	X_Point;	//X坐标
	uint16_t	Y_Point;	//Y坐标
	uint16_t	S_Point;	//触摸点大小
}XY_Coordinate;	//触摸点坐标


/*触摸结构体*/
typedef struct
{
	uint8_t Touch_State				;	//触摸状态
	uint8_t Touch_Number			;	//触摸数量
	XY_Coordinate Touch_XY[GT_TOUCH_MAX]	;	//触摸的x坐标，对于gt911最多5个点的坐标
}Touch_Struct;	//触摸信息结构体

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
	
//	GPIOC->CRL&=0XFFFF0FF0;	//PC0,PC3 推挽输出
//	GPIOC->CRL|=0X00003003;	   
//	GPIOC->ODR|=1<<1;	    //PC0 输出高	 
//	GPIOC->ODR|=1<<3;	    //PC3 输出高	 
}



void CT_IIC_Start(void)
{
	CT_SDA_OUT();
	SETBIT_SDA(1);	  	  
	SETBIT_SCL(1);
	CT_Delay();
 	SETBIT_SDA(0);//START:when CLK is high,DATA change form high to low 
	CT_Delay();
	SETBIT_SCL(0);//钳住I2C总线，准备发送或接收数据 
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
	CT_SDA_IN();      //SDA设置为输入  
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
	SETBIT_SCL(0);//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
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
//不产生ACK应答		    
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void CT_IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	CT_SDA_OUT(); 	    
    SETBIT_SCL(0);//拉低时钟开始数据传输
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
 	CT_SDA_IN();//SDA设置为输入
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
	if (!ack)CT_IIC_NAck();//发送nACK
	else CT_IIC_Ack(); //发送ACK   
 	return receive;
}

uint8_t GT911_WR_Reg(uint16_t reg,uint8_t *buf,uint8_t len)
{
	uint8_t i;
	uint8_t ret=0;
	CT_IIC_Start();	
 	CT_IIC_Send_Byte(GT_CMD_WR);   	//发送写命令 	 
	CT_IIC_Wait_Ack();
	CT_IIC_Send_Byte(reg>>8);   	//发送高8位地址
	CT_IIC_Wait_Ack(); 	 										  		   
	CT_IIC_Send_Byte(reg&0XFF);   	//发送低8位地址
	CT_IIC_Wait_Ack();  
	for(i=0;i<len;i++)
	{	   
    	CT_IIC_Send_Byte(buf[i]);  	//发数据
		ret=CT_IIC_Wait_Ack();
		if(ret)break;  
	}
    CT_IIC_Stop();					//产生一个停止条件	    
	return ret; 
}
//从GT911读出一次数据
//reg:起始寄存器地址
//buf:数据缓缓存区
//len:读数据长度			  
void GT911_RD_Reg(uint16_t reg,uint8_t *buf,uint8_t len)
{
	uint8_t i; 
 	CT_IIC_Start();	
 	CT_IIC_Send_Byte(GT_CMD_WR);   //发送写命令 	 
	CT_IIC_Wait_Ack();
 	CT_IIC_Send_Byte(reg>>8);   	//发送高8位地址
	CT_IIC_Wait_Ack(); 	 										  		   
 	CT_IIC_Send_Byte(reg&0XFF);   	//发送低8位地址
	CT_IIC_Wait_Ack();  
 	CT_IIC_Start();  	 	   
	CT_IIC_Send_Byte(GT_CMD_RD);   //发送读命令		   
	CT_IIC_Wait_Ack();	
    
	for(i=0;i<len;i++)
	{	   
    	buf[i]=CT_IIC_Read_Byte(i==(len-1)?0:1); //发数据	  
	} 
    CT_IIC_Stop();//产生一个停止条件    
} 
//初始化GT911触摸屏
//返回值:0,初始化成功;1,初始化失败 



uint8_t GT911_Init(void)
{
    
    
    /*
    TP_PEN    PA7    CT_INT   15
    SPI1_CS   PC4    CT_RST   14
    */
	uint8_t temp[5]={0}; 
	GPIO_InitTypeDef  GPIO_InitStructure;	

	GPIO_InitStructure.Pin =GPIO_PIN_7;// pa7  int
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;	//输入
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);  //int拉高
    
    
	GPIO_InitStructure.Pin =GPIO_PIN_4;// pc4 rst
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;	//推挽输出
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);  //rst拉高
	

 	CT_IIC_Init();  //初始化电容屏的I2C总线  
	GT_RST(0);				//复位
	 HAL_Delay(10);
 	GT_RST(1);				//释放复位 
 	 HAL_Delay(10);
	
GPIO_InitStructure.Pin =GPIO_PIN_4;// 15端口配置  rst
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;	//推挽输出
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

 	 HAL_Delay(10);
	GT911_RD_Reg(GT_PID_REG,temp,4);//读取产品ID
	temp[4]=0;

	if(strcmp((char*)temp,"911"))//ID==911
	{
		temp[0]=0X02;			
		GT911_WR_Reg(GT_CTRL_REG,temp,1);//软复位GT911
		GT911_RD_Reg(GT_CFGS_REG,temp,1);//读取GT_CFGS_REG寄存器
//		if(temp[0]<0X60)//默认版本比较低,需要更新flash配置
//		{
//			printf("Default Ver:%d\r\n",temp[0]);
//		}
		CT_Delay();
		temp[0]=0X00;	 
		GT911_WR_Reg(GT_CTRL_REG,temp,1);//结束复位   
		return 0;
	} 
    
	
	return 1;
    
}




// GT911驱动配置，默认使用第一份配置文件，如果屏幕异常，可尝试使用第二份
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
   /*初始化gt9157的设备地址为0x28/0x29*/

   HAL_GPIO_WritePin (GTP_INT_GPIO_PORT,GTP_INT_GPIO_PIN,GPIO_PIN_RESET);
   HAL_GPIO_WritePin (GTP_RST_GPIO_PORT,GTP_RST_GPIO_PIN,GPIO_PIN_RESET);
   HAL_Delay(100);
   /*复位为低电平，为初始化做准备*/
   HAL_GPIO_WritePin (GTP_INT_GPIO_PORT,GTP_INT_GPIO_PIN,GPIO_PIN_SET);
   HAL_Delay(10);

   /*拉高一段时间，进行初始化*/
   HAL_GPIO_WritePin (GTP_RST_GPIO_PORT,GTP_RST_GPIO_PIN,GPIO_PIN_SET);
    HAL_Delay(10);

   /*把INT引脚设置为浮空输入模式，以便接收触摸中断信号*/
   GPIO_InitTypeDef GPIO_InitStructure;
   GPIO_InitStructure.Pin = GTP_INT_GPIO_PIN;
   GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
   GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
   GPIO_InitStructure.Pull  = GPIO_NOPULL;
   HAL_GPIO_Init(GTP_INT_GPIO_PORT, &GPIO_InitStructure);
   
   HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 2);/* 配置中断优先级 */
   HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);/* 使能中断 */
HAL_GPIO_WritePin (GTP_RST_GPIO_PORT,GTP_RST_GPIO_PIN,GPIO_PIN_RESET);//下拉
HAL_GPIO_WritePin (GTP_INT_GPIO_PORT,GTP_INT_GPIO_PIN,GPIO_PIN_RESET);//下拉

    HAL_Delay(100);
   uint8_t GTP_ID[4];
   GT911_RD_Reg(GT_PID_REG,GTP_ID,4);
  

   // 写入配置表
   GT911_WR_Reg(GT_CFGS_REG, (uint8_t *)CTP_CFG_GT911, sizeof(CTP_CFG_GT911));
   HAL_Delay(100);

   // 转换为软复位模式
   uint8_t t_temp=2;	//中间变量
   GT911_WR_Reg(GT_CTRL_REG, &t_temp, 1);
   HAL_Delay(100);

   // 转换为读取坐标模式
   t_temp=0;	//中间变量
   GT911_WR_Reg(GT_CTRL_REG, &t_temp, 1);

}



/*
	功能：gt911触摸扫描，判断当前是否被触摸
	参数1：
*/
void GTXXXX_Scanf(void)
{
	uint8_t t_temp;	//中间变量

	GT911_RD_Reg(GT_GSTID_REG, &t_temp, 1);//读取状态寄存器

	// 记录触摸状态
	User_Touch.Touch_State = t_temp;
	User_Touch.Touch_Number = (User_Touch.Touch_State & 0x0f);	//获取触摸点数
	User_Touch.Touch_State = (User_Touch.Touch_State & 0x80);	//触摸状态

	//判断是否有触摸数据
	switch(User_Touch.Touch_State)
	{
		case TOUCH__NO:		//没有数据
			break;
		case TOUCH_ING:		//触摸中~后，有数据，并读出数据
			for(uint8_t i=0; i<User_Touch.Touch_Number; i++)
			{
				GT911_RD_Reg((GT_TPD_Sta + i*8 + X_L), &t_temp, 1);	//读出触摸x坐标的低8位
				User_Touch.Touch_XY[i].X_Point  = t_temp;
				GT911_RD_Reg((GT_TPD_Sta + i*8 + X_H), &t_temp, 1);	//读出触摸x坐标的高8位
				User_Touch.Touch_XY[i].X_Point |= (t_temp<<8);

				GT911_RD_Reg((GT_TPD_Sta + i*8 + Y_L), &t_temp, 1);	//读出触摸y坐标的低8位
				User_Touch.Touch_XY[i].Y_Point  = t_temp;
				GT911_RD_Reg((GT_TPD_Sta + i*8 + Y_H), &t_temp, 1);	//读出触摸y坐标的高8位
				User_Touch.Touch_XY[i].Y_Point |= (t_temp<<8);

				GT911_RD_Reg((GT_TPD_Sta + i*8 + S_L), &t_temp, 1);	//读出触摸大小数据的低8位
				User_Touch.Touch_XY[i].S_Point  = t_temp;
				GT911_RD_Reg((GT_TPD_Sta + i*8 + S_H), &t_temp, 1);	//读出触摸大小数据的高8位
				User_Touch.Touch_XY[i].S_Point |= (t_temp<<8);
			}

			t_temp=0;
			GT911_WR_Reg(GT_GSTID_REG, &t_temp, 1);	//清除数据标志位
		break;
	}
}

void GTP911_Test(void)
{
	GTXXXX_Scanf();		//不断扫描
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

