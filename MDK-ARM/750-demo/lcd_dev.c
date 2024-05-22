
#include "main.h"

#define LCD_REGION_NUMBER		MPU_REGION_NUMBER0
#define LCD_ADDRESS_START		(0X60000000)	
#define LCD_REGION_SIZE			MPU_REGION_SIZE_256MB 

static uint8_t  fac_us=0;//us—” ±±∂≥À ˝
static uint16_t fac_ms=0;//ms—” ±±∂≥À ˝


typedef struct
{
    volatile uint16_t LCD_REG;
    volatile uint16_t LCD_RAM;
} LCD_TypeDef;

typedef struct  
{										    
	uint16_t width;			//LCD ????
	uint16_t height;			//LCD ???
	uint16_t id;				  //LCD ID
	uint8_t  dir;			  //?????????????????0????????1????????	
	uint8_t	wramcmd;		//??'?gram???
	uint8_t  setxcmd;		//????x???????
	uint8_t  setycmd;		//????y???????	 
}_lcd_dev; 

#define L2R_U2D  0 //??????,???????
#define L2R_D2U  1 //??????,???µ???
#define R2L_U2D  2 //???????,???????
#define R2L_D2U  3 //???????,???µ???

#define U2D_L2R  4 //???????,??????
#define U2D_R2L  5 //???????,???????
#define D2U_L2R  6 //???µ???,??????
#define D2U_R2L  7 //???µ???,???????	

#define RGB 0x00
#define BGR 0x08
#define COLORORDER BGR
#define Landscape 1//
#define LCD_WIDTH  320
#define LCD_HEIGHT 480

#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	   0x001F  
#define BRED             0XF81F
#define GRED 			 	     0XFFE0
#define GBLUE			       0X07FF
#define RED           	 0xF800
#define GREEN         	 0x07E0

uint8_t DFT_SCAN_DIR; //…®√Ë∑ΩœÚ±‰¡ø

#define	LCD_RST(x) HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin, x)
#define	LCD_LED(x) HAL_GPIO_WritePin(GPIOC, LCD_LED_Pin, x)
//#define	LCD_RS(x) HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, x)
//#define	LCD_CS(x) HAL_GPIO_WritePin(GPIOD, LCD_CS_Pin, x)
//#define	LCD_WR(x) HAL_GPIO_WritePin(GPIOD, LCD_WR_Pin, x)
//#define	LCD_RD(x) HAL_GPIO_WritePin(GPIOD, LCD_RD_Pin, x)

//#define DATAOUT(x) {HAL_GPIO_WritePin(GPIOD, LCD_D13_Pin|LCD_D14_Pin|LCD_D15_Pin, x)}//{GPIOB->ODR=(GPIOB->ODR&0X00FF)|((uint16_t)(x<<8)&0xFF00)}

#define LCD_FMC_NEX         1    
#define LCD_FMC_AX          19
#define LCD_BASE        (uint32_t)((0X60000000 + (0X4000000 * (LCD_FMC_NEX - 1))) | (((1 << LCD_FMC_AX) * 2) -2))
#define LCD             ((LCD_TypeDef *) LCD_BASE)

void LCD_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color);
void LCD_WR_REG(uint16_t regno);
void LCD_WR_DATA(uint16_t data);
void LCD_Clear(uint16_t color);

//π‹¿ÌLCD÷ÿ“™≤Œ ˝
//ƒ¨»œŒ™ ˙∆¡
_lcd_dev lcddev;
					    
void ct_delay_us(uint32_t nus)
  {
      uint32_t ticks;
    uint32_t told,tnow,tcnt=0;
    uint32_t reload=SysTick->LOAD;
    ticks=nus*fac_us; 
    told=SysTick->VAL; 
    while(1)
    {
        tnow=SysTick->VAL;
        if(tnow!=told)
        {
            if(tnow<told)tcnt+=told-tnow;
            else tcnt+=reload-tnow+told;
            told=tnow;
            if(tcnt>=ticks)break; 
        }
    };
}
  

void CT_Delay(void)
{
	ct_delay_us(2);
} 

void LCD_Set_Window(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height)
{   
	width=sx+width-1;
	height=sy+height-1;

	LCD_WR_REG(lcddev.setxcmd);
	LCD_WR_DATA(sx>>8);  
	LCD_WR_DATA(sx&0XFF);	  
	LCD_WR_DATA(width>>8);   
	LCD_WR_DATA(width&0XFF);   
	LCD_WR_REG(lcddev.setycmd);
	LCD_WR_DATA(sy>>8);   
	LCD_WR_DATA(sy&0XFF);  
	LCD_WR_DATA(height>>8);   
	LCD_WR_DATA(height&0XFF);  
}


void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{	
	LCD_WR_REG(LCD_Reg);
	LCD_WR_DATA(LCD_RegValue);
}	

void LCD_Scan_Dir(uint8_t dir)
{
	uint16_t regval=0;
	uint8_t dirreg=0;
//	uint16_t temp;  
	switch(dir)//∑ΩœÚ◊™ªª
	{
		case 0:dir=6;break;
		case 1:dir=7;break;
		case 2:dir=4;break;
		case 3:dir=5;break;
		case 4:dir=1;break;
		case 5:dir=0;break;
		case 6:dir=3;break;
		case 7:dir=2;break;	     
	}
	switch(dir)
	{
		case L2R_U2D://¥”◊ÛµΩ”“,¥”…œµΩœ¬
			regval|=(0<<7)|(0<<6)|(0<<5); 
			break;
		case L2R_D2U://¥”◊ÛµΩ”“,¥”œ¬µΩ…œ
			regval|=(1<<7)|(0<<6)|(0<<5); 
			break;
		case R2L_U2D://¥””“µΩ◊Û,¥”…œµΩœ¬
			regval|=(0<<7)|(1<<6)|(0<<5); 
			break;
		case R2L_D2U://¥””“µΩ◊Û,¥”œ¬µΩ…œ
			regval|=(1<<7)|(1<<6)|(0<<5); 
			break;	 
		case U2D_L2R://¥”…œµΩœ¬,¥”◊ÛµΩ”“
			regval|=(0<<7)|(0<<6)|(1<<5); 
			break;
		case U2D_R2L://¥”…œµΩœ¬,¥””“µΩ◊Û
			regval|=(0<<7)|(1<<6)|(1<<5); 
			break;
		case D2U_L2R://¥”œ¬µΩ…œ,¥”◊ÛµΩ”“
			regval|=(1<<7)|(0<<6)|(1<<5); 
			break;
		case D2U_R2L://¥”œ¬µΩ…œ,¥””“µΩ◊Û
			regval|=(1<<7)|(1<<6)|(1<<5); 
			break;	 
	}
	dirreg=0X36; 
  regval|=COLORORDER;	//0x08 0x00  ∫Ï¿∂∑¥…´ø…“‘Õ®π˝’‚¿Ô–ﬁ∏ƒ
	LCD_WriteReg(dirreg,regval);
			
	LCD_WR_REG(lcddev.setxcmd); 
	LCD_WR_DATA(0);LCD_WR_DATA(0);
	LCD_WR_DATA((lcddev.width-1)>>8);LCD_WR_DATA((lcddev.width-1)&0XFF);
	LCD_WR_REG(lcddev.setycmd); 
	LCD_WR_DATA(0);LCD_WR_DATA(0);
	LCD_WR_DATA((lcddev.height-1)>>8);LCD_WR_DATA((lcddev.height-1)&0XFF);  
		
  	
}

void LCD_Display_Dir(uint8_t dir)
{
	if(dir==0)			 
	{
		lcddev.dir=0;	// ˙∆¡
		lcddev.width=LCD_WIDTH;
		lcddev.height=LCD_HEIGHT;

		lcddev.wramcmd=0X2C;
		lcddev.setxcmd=0X2A;
		lcddev.setycmd=0X2B;  
    DFT_SCAN_DIR=D2U_R2L;	    // ˙œ‘-…Ë∂®œ‘ æ∑ΩœÚ	

	}else 				  //∫·∆¡
	{	  				
		lcddev.dir=1;	 
		lcddev.width=LCD_HEIGHT;
		lcddev.height=LCD_WIDTH;

		lcddev.wramcmd=0X2C;
		lcddev.setxcmd=0X2A;
		lcddev.setycmd=0X2B;  
    DFT_SCAN_DIR=L2R_D2U;     //∫·œ‘-…Ë∂®œ‘ æ∑ΩœÚ		
		
	} 
	LCD_Scan_Dir(DFT_SCAN_DIR);	//ƒ¨»œ…®√Ë∑ΩœÚ
}

static void lcd_mpu_config(void)
{	

	MPU_Region_InitTypeDef MPU_Initure;

	HAL_MPU_Disable();

	MPU_Initure.Enable=MPU_REGION_ENABLE;

	MPU_Initure.Number=LCD_REGION_NUMBER;

	MPU_Initure.BaseAddress=LCD_ADDRESS_START;//0x60000000

	MPU_Initure.Size=LCD_REGION_SIZE;

	MPU_Initure.SubRegionDisable=0X00;

	MPU_Initure.TypeExtField=MPU_TEX_LEVEL0;

	MPU_Initure.AccessPermission=MPU_REGION_FULL_ACCESS;

	MPU_Initure.DisableExec=MPU_INSTRUCTION_ACCESS_ENABLE;

	MPU_Initure.IsShareable=MPU_ACCESS_NOT_SHAREABLE;

	MPU_Initure.IsCacheable=MPU_ACCESS_NOT_CACHEABLE;

	MPU_Initure.IsBufferable=MPU_ACCESS_BUFFERABLE;

	HAL_MPU_ConfigRegion(&MPU_Initure);

    

    /* ??MPU */

	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

void DATAOUT(uint16_t regval)
{
	
//	for(int i=0; i<16; i++)
//	{
//		switch(i)
//		{
//			case 0:
//				if(regval&0x01)
//					HAL_GPIO_WritePin(GPIOD, LCD_D0_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(GPIOD, LCD_D0_Pin, GPIO_PIN_RESET);
//				break;
//			case 1:
//				if(regval&0x02)
//					HAL_GPIO_WritePin(GPIOD, LCD_D1_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(GPIOD, LCD_D1_Pin, GPIO_PIN_RESET);
//				break;
//			case 2:
//				if(regval&0x04)
//					HAL_GPIO_WritePin(GPIOD, LCD_D2_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(GPIOD, LCD_D2_Pin, GPIO_PIN_RESET);
//				break;
//			case 3:
//				if(regval&0x08)
//					HAL_GPIO_WritePin(GPIOD, LCD_D3_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(GPIOD, LCD_D3_Pin, GPIO_PIN_RESET);
//				break;
//			case 4:
//				if(regval&0x10)
//					HAL_GPIO_WritePin(GPIOE, LCD_D4_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(GPIOE, LCD_D4_Pin, GPIO_PIN_RESET);
//				break;
//			case 5:
//				if(regval&0x20)
//					HAL_GPIO_WritePin(GPIOE, LCD_D5_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(GPIOE, LCD_D5_Pin, GPIO_PIN_RESET);
//				break;
//			case 6:
//				if(regval&0x40)
//					HAL_GPIO_WritePin(GPIOE, LCD_D6_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(GPIOE, LCD_D6_Pin, GPIO_PIN_RESET);
//				break;
//			case 7:
//				if(regval&0x80)
//					HAL_GPIO_WritePin(GPIOE, LCD_D7_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(GPIOE, LCD_D7_Pin, GPIO_PIN_RESET);
//				break;
//				
//				
//				case 8:
//				if(regval&0x0100)
//					HAL_GPIO_WritePin(GPIOE, LCD_D8_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(GPIOE, LCD_D8_Pin, GPIO_PIN_RESET);
//				break;
//			case 9:
//				if(regval&0x0200)
//					HAL_GPIO_WritePin(GPIOE, LCD_D9_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(GPIOE, LCD_D9_Pin, GPIO_PIN_RESET);
//				break;
//			case 10:
//				if(regval&0x0400)
//					HAL_GPIO_WritePin(GPIOE, LCD_D10_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(GPIOE, LCD_D10_Pin, GPIO_PIN_RESET);
//				break;
//			case 11:
//				if(regval&0x0800)
//					HAL_GPIO_WritePin(GPIOE, LCD_D11_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(GPIOE, LCD_D11_Pin, GPIO_PIN_RESET);
//				break;
//			case 12:
//				if(regval&0x1000)
//					HAL_GPIO_WritePin(GPIOE, LCD_D12_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(GPIOE, LCD_D12_Pin, GPIO_PIN_RESET);
//				break;
//			case 13:
//				if(regval&0x2000)
//					HAL_GPIO_WritePin(GPIOD, LCD_D13_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(GPIOD, LCD_D13_Pin, GPIO_PIN_RESET);
//				break;
//			case 14:
//				if(regval&0x4000)
//					HAL_GPIO_WritePin(GPIOD, LCD_D14_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(GPIOD, LCD_D14_Pin, GPIO_PIN_RESET);
//				break;
//			case 15:
//				if(regval&0x8000)
//					HAL_GPIO_WritePin(GPIOD, LCD_D15_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(GPIOD, LCD_D15_Pin, GPIO_PIN_RESET);
//				break;
//		}
//	}
}

//regval:ºƒ¥Ê∆˜÷µ  
	void LCD_WR_REG(uint16_t regno)
	{
		//regno = regno;          
    LCD->LCD_REG = regno; 		
//		HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);//LCD_RS(0);
//		HAL_GPIO_WritePin(GPIOD, LCD_CS_Pin, GPIO_PIN_RESET);//LCD_CS(0);
//		DATAOUT(regval); 
//		HAL_GPIO_WritePin(GPIOD, LCD_WR_Pin, GPIO_PIN_RESET);//LCD_WR(0);
//		HAL_GPIO_WritePin(GPIOD, LCD_WR_Pin, GPIO_PIN_SET);//LCD_WR(1);
//		HAL_GPIO_WritePin(GPIOD, LCD_CS_Pin, GPIO_PIN_SET);//LCD_CS(1);
//		LCD_RS=0;//–¥µÿ÷∑ 
//		LCD_CS=0; 
//		DATAOUT(regval); 
//		LCD_WR=0; 
//		LCD_WR=1; 
//		LCD_CS=1;	
	}
	//–¥LCD ˝æ›
	//data:“™–¥»Îµƒ÷µ
	void LCD_WR_DATA(uint16_t data)
	{	
		//data = data;           
    LCD->LCD_RAM = data;
//		HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);//LCD_RS(1);
//		HAL_GPIO_WritePin(GPIOD, LCD_CS_Pin, GPIO_PIN_RESET);//LCD_CS(0);
//		DATAOUT(data); 
//		HAL_GPIO_WritePin(GPIOD, LCD_WR_Pin, GPIO_PIN_RESET);//LCD_WR(0);
//		HAL_GPIO_WritePin(GPIOD, LCD_WR_Pin, GPIO_PIN_SET);//LCD_WR(1);
//		HAL_GPIO_WritePin(GPIOD, LCD_CS_Pin, GPIO_PIN_SET);//LCD_CS(1);		
//		LCD_RS=1;
//		LCD_CS=0;
//		DATAOUT(data);
//		LCD_WR=0;
//		LCD_WR=1;
//		LCD_CS=1;
	}	
	//LCD–¥GRAM
	//RGB_Code:—’…´÷µ
	void LCD_WriteRAM(uint16_t RGB_Code)
	{		
//		LCD_RS(1);
//		LCD_CS(0);
		//RGB_Code = RGB_Code; 
		LCD->LCD_RAM = RGB_Code;
		//DATAOUT(RGB_Code); 
//		LCD_WR(0);
//		LCD_WR(1);
//		LCD_CS(1);		
//		LCD_RS=1;
//		LCD_CS=0;
//		DATAOUT(RGB_Code);
//		LCD_WR=0;
//		LCD_WR=1;
//		LCD_CS=1;
	}
	
//LCD≥ı ºªØ∑≈‘⁄’‚∏ˆ∫Ø ˝÷–
void LCD_INIT_CODE(void)
{ 
	lcd_mpu_config();
	HAL_GPIO_WritePin(GPIOC, LCD_LED_Pin, GPIO_PIN_SET);//LCD_LED(1);
	HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin, GPIO_PIN_RESET);//LCD_RST(0);
	HAL_Delay(20);
	HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin, GPIO_PIN_SET);//LCD_RST(1);
	HAL_Delay(20);
	//HAL_GPIO_WritePin(GPIOD, LCD_RD_Pin, GPIO_PIN_SET);//LCD_RD(1);
	
	LCD_WR_REG(0x11);
	HAL_Delay(120);

	LCD_WR_REG(0Xf0);
	LCD_WR_DATA(0xc3);
	LCD_WR_REG(0Xf0);
	LCD_WR_DATA(0x96);
	LCD_WR_REG(0X36); 
	LCD_WR_DATA(0x48);     //??????
	LCD_WR_REG(0Xb4);
	LCD_WR_DATA(0x01);	  //01
	LCD_WR_REG(0x3a);
	LCD_WR_DATA(0x05);

	LCD_WR_REG(0Xe8);
	LCD_WR_DATA(0x40);
	LCD_WR_DATA(0x82);
	LCD_WR_DATA(0x07);
	LCD_WR_DATA(0x18);
	LCD_WR_DATA(0x27);
	LCD_WR_DATA(0x0a);
	LCD_WR_DATA(0xb6);
	LCD_WR_DATA(0x33);

	//LCD_WR_REG(0xc1);
	//LCD_WR_DATA(0x0f);//06 ????????

	LCD_WR_REG(0Xc5);	 //18 ????????
	LCD_WR_DATA(0x23);	 //23

	LCD_WR_REG(0Xc2);	
	LCD_WR_DATA(0xa7);

	LCD_WR_REG(0Xe0);
	LCD_WR_DATA(0xf0);
	LCD_WR_DATA(0x01);
	LCD_WR_DATA(0x06);
	LCD_WR_DATA(0x0f);
	LCD_WR_DATA(0x12);
	LCD_WR_DATA(0x1d);
	LCD_WR_DATA(0x36);
	LCD_WR_DATA(0x54);
	LCD_WR_DATA(0x44);
	LCD_WR_DATA(0x0c);
	LCD_WR_DATA(0x18);
	LCD_WR_DATA(0x16);
	LCD_WR_DATA(0x13);
	LCD_WR_DATA(0x15);

	LCD_WR_REG(0Xe1);
	LCD_WR_DATA(0xf0);
	LCD_WR_DATA(0x01);
	LCD_WR_DATA(0x05);
	LCD_WR_DATA(0x0a);
	LCD_WR_DATA(0x0b);
	LCD_WR_DATA(0x07);
	LCD_WR_DATA(0x32);
	LCD_WR_DATA(0x44);
	LCD_WR_DATA(0x44);
	LCD_WR_DATA(0x0c);
	LCD_WR_DATA(0x18);
	LCD_WR_DATA(0x17);
	LCD_WR_DATA(0x13);
	LCD_WR_DATA(0x16);

	LCD_WR_REG(0Xf0);
	LCD_WR_DATA(0x3c);

	LCD_WR_REG(0Xf0);
	LCD_WR_DATA(0x69);
	HAL_Delay(120);

	LCD_WR_REG(0X29);
	HAL_Delay(120);
	//LCD_Fill(0,0,200,200,0X001F);	
	
	LCD_Display_Dir(Landscape);		 //1: ˙∆¡£ª0:∫·∆¡   ∫· ˙∆¡¥”’‚¿Ô«–ª
	
	LCD_Clear(WHITE);//À¢∞◊
    HAL_Delay(20);
    LCD_Clear(BLACK);//À¢∞◊
    HAL_Delay(20);
    LCD_Clear(BLUE);//À¢∞◊
    HAL_Delay(20);
    LCD_Clear(WHITE);//À¢∞◊
	LCD_Clear(BLACK);//À¢∞◊
    
		HAL_Delay(50);
	
    
}

void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{	 
	LCD_WR_REG(lcddev.setxcmd); 
	LCD_WR_DATA(Xpos>>8); 
	LCD_WR_DATA(Xpos&0XFF);
	LCD_WR_DATA(Xpos>>8); 
	LCD_WR_DATA(Xpos&0XFF);
  	
	LCD_WR_REG(lcddev.setycmd); 
	LCD_WR_DATA(Ypos>>8); 
	LCD_WR_DATA(Ypos&0XFF);
	LCD_WR_DATA(Ypos>>8); 
	LCD_WR_DATA(Ypos&0XFF);
	
	//LCD_WR_REG(0X2C);
} 

void LCD_WriteRAM_Prepare(void)
{
 	LCD_WR_REG(lcddev.wramcmd);
}	

void LCD_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color)
{          
	uint16_t i,j;
	uint16_t xlen=0;
	xlen=ex-sx+1;	   
	for(i=sy;i<=ey;i++)
	{
	 	LCD_SetCursor(sx,i);      				//…Ë÷√π‚±ÍŒª÷√ 
		LCD_WriteRAM_Prepare();     			//ø™ º–¥»ÎGRAM	  
		for(j=0;j<xlen;j++)
			LCD_WriteRAM(color);	//…Ë÷√π‚±ÍŒª÷√ 	    
	}
} 

void LCD_Clear(uint16_t color)
{
  uint32_t index=0;      
	uint32_t totalpoint=lcddev.width;
	totalpoint*=lcddev.height; 	//µ√µΩ◊‹µ„ ˝
  LCD_Set_Window(0,0,lcddev.width,lcddev.height);
	LCD_WriteRAM_Prepare();     //ø™ º–¥»ÎGRAM	

  
	for(index=0; index<totalpoint; index++)
	{
		//LCD_WriteRAM(color);
		LCD->LCD_RAM = color;
	}
} 



