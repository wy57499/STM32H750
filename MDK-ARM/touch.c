
#include "touch.h"
extern u8 AUTO;
u16 x0temp,y0temp;
//������ز���
_m_tp_dev tp_dev=
{
	TP_Init,
	TP_Scan,
	TP_Adjust,
	0,
	0, 
	0,
	0,
	0,
	0,	  	 		
	0,
	0,	  	 		
};					
//Ĭ��Ϊtouchtype=0������.
u8 CMD_RDX=0XD0;
u8 CMD_RDY=0X90;
 	 			    					   
//SPIд����
//������ICд��1byte����    
//num:Ҫд�������
void TP_Write_Byte(u8 num)    
{  
	u8 count=0;   
	for(count=0;count<8;count++)  
	{ 	  
		if(num&0x80)TDIN=1;  
		else TDIN=0;   
		num<<=1;    
		TCLK=0; 
		delay_us(1);
		TCLK=1;		//��������Ч	        
	}		 			    
} 		 
//SPI������ 
//�Ӵ�����IC��ȡadcֵ
//CMD:ָ��
//����ֵ:����������	   
u16 TP_Read_AD(u8 CMD)	  
{ 	 
	u8 count=0; 	  
	u16 Num=0; 
	TCLK=0;		//������ʱ�� 	 
	TDIN=0; 	//����������
	TCS=0; 		//ѡ�д�����IC
	TP_Write_Byte(CMD);//����������
	delay_us(6);//ADS7846��ת��ʱ���Ϊ6us
	TCLK=0; 	     	    
	delay_us(1);    	   
	TCLK=1;		//��1��ʱ�ӣ����BUSY
	delay_us(1);    
	TCLK=0; 	     	    
	for(count=0;count<16;count++)//����16λ����,ֻ�и�12λ��Ч 
	{ 				  
		Num<<=1; 	 
		TCLK=0;	//�½�����Ч  	    	   
		delay_us(1);    
 		TCLK=1;
 		if(DOUT)Num++; 		 
	}  	
	Num>>=4;   	//ֻ�и�12λ��Ч.
	TCS=1;		//�ͷ�Ƭѡ	 
	return(Num);   
}
//��ȡһ������ֵ(x����y)
//������ȡREAD_TIMES������,����Щ������������,
//Ȼ��ȥ����ͺ����LOST_VAL����,ȡƽ��ֵ 
//xy:ָ�CMD_RDX/CMD_RDY��
//����ֵ:����������
#define READ_TIMES 5 	//��ȡ����
#define LOST_VAL 1	  	//����ֵ
u16 TP_Read_XOY(u8 xy)
{
	u16 i, j;
	u16 buf[READ_TIMES];
	u16 sum=0;
	u16 temp;
	for(i=0;i<READ_TIMES;i++)buf[i]=TP_Read_AD(xy);		 		    
	for(i=0;i<READ_TIMES-1; i++)//����
	{
		for(j=i+1;j<READ_TIMES;j++)
		{
			if(buf[i]>buf[j])//��������
			{
				temp=buf[i];
				buf[i]=buf[j];
				buf[j]=temp;
			}
		}
	}	  
	sum=0;
	for(i=LOST_VAL;i<READ_TIMES-LOST_VAL;i++)sum+=buf[i];
	temp=sum/(READ_TIMES-2*LOST_VAL);
	return temp;   
} 
//��ȡx,y����
//��Сֵ��������100.
//x,y:��ȡ��������ֵ
//����ֵ:0,ʧ��;1,�ɹ���
u8 TP_Read_XY(u16 *x,u16 *y)
{
	u16 xtemp,ytemp;			 	 		  
	xtemp=TP_Read_XOY(CMD_RDX);
	ytemp=TP_Read_XOY(CMD_RDY);	  												   
	//if(xtemp<100||ytemp<100)return 0;//����ʧ��
	x0temp=xtemp;
	y0temp=ytemp;//��ֵ���ݵ�ǰ�������ж��Ƿ��д�����
	
	*x=xtemp;
	*y=ytemp;
	return 1;//�����ɹ�
}
//����2�ζ�ȡ������IC,�������ε�ƫ��ܳ���
//ERR_RANGE,��������,����Ϊ������ȷ,�����������.	   
//�ú����ܴ�����׼ȷ��
//x,y:��ȡ��������ֵ
//����ֵ:0,ʧ��;1,�ɹ���
#define ERR_RANGE 50 //��Χ 
u8 TP_Read_XY2(u16 *x,u16 *y) 
{
	u16 x1,y1;
 	u16 x2,y2;
 	u8 flag;    
    flag=TP_Read_XY(&x1,&y1);   
    if(flag==0)return(0);
    flag=TP_Read_XY(&x2,&y2);	   
    if(flag==0)return(0);   
    if(((x2<=x1&&x1<x2+ERR_RANGE)||(x1<=x2&&x2<x1+ERR_RANGE))//ǰ�����β�����+-50��
    &&((y2<=y1&&y1<y2+ERR_RANGE)||(y1<=y2&&y2<y1+ERR_RANGE)))
    {
        *x=(x1+x2)/2;
        *y=(y1+y2)/2;
        return 1;
    }else return 0;	  
}  
//////////////////////////////////////////////////////////////////////////////////		  
//��LCD�����йصĺ���  
//��һ��������
//����У׼�õ�
//x,y:����
//color:��ɫ
void TP_Drow_Touch_Point(u16 x,u16 y,u16 color)
{
	POINT_COLOR=color;
	LCD_DrawLine(x-12,y,x+13,y,color);//����
	LCD_DrawLine(x,y-12,x,y+13,color);//����
	LCD_DrawPoint(x+1,y+1,color);
	LCD_DrawPoint(x-1,y+1,color);
	LCD_DrawPoint(x+1,y-1,color);
	LCD_DrawPoint(x-1,y-1,color);
	Draw_Circle(x,y,6,color);//������Ȧ
}	  
//��һ�����(2*2�ĵ�)		   
//x,y:����
//color:��ɫ
void TP_Draw_Big_Point(u16 x,u16 y,u16 color)
{	    
	POINT_COLOR=color;
	LCD_DrawPoint(x,y,color);//���ĵ� 
	LCD_DrawPoint(x+1,y,color);
	LCD_DrawPoint(x,y+1,color);
	LCD_DrawPoint(x+1,y+1,color);	 	  	
}						  
//////////////////////////////////////////////////////////////////////////////////		  
//��������ɨ��
//tp:0,��Ļ����;1,��������(У׼�����ⳡ����)
//����ֵ:��ǰ����״̬.
//0,�����޴���;1,�����д���
u8 TP_Scan(u8 tp)
{			   
	if(PEN==0)//�а�������
	{
		if(tp)TP_Read_XY2(&tp_dev.x[0],&tp_dev.y[0]);//��ȡ��������
		else if(TP_Read_XY2(&tp_dev.x[0],&tp_dev.y[0]))//��ȡ��Ļ����
		{
	 		tp_dev.x[0]=tp_dev.xfac*tp_dev.x[0]+tp_dev.xoff;//�����ת��Ϊ��Ļ����
			tp_dev.y[0]=tp_dev.yfac*tp_dev.y[0]+tp_dev.yoff;  
	 	} 
		if((tp_dev.sta&TP_PRES_DOWN)==0)//֮ǰû�б�����
		{		 
			tp_dev.sta=TP_PRES_DOWN|TP_CATH_PRES;//��������  
			tp_dev.x[4]=tp_dev.x[0];//��¼��һ�ΰ���ʱ������
			tp_dev.y[4]=tp_dev.y[0];  	   			 
		}
		
		printf("x:%d\r\n",tp_dev.x[0]);	
		printf("y:%d\r\n",tp_dev.y[0]);	  //����TP����ʱʹ��
				   
	}else
	{
		if(tp_dev.sta&TP_PRES_DOWN)//֮ǰ�Ǳ����µ�
		{
			tp_dev.sta&=~(1<<7);//��ǰ����ɿ�	
		}else//֮ǰ��û�б�����
		{
			tp_dev.x[4]=0;
			tp_dev.y[4]=0;
			tp_dev.x[0]=0xffff;
			tp_dev.y[0]=0xffff;
		}	    
	}
	return tp_dev.sta&TP_PRES_DOWN;//���ص�ǰ�Ĵ���״̬
}	  
//////////////////////////////////////////////////////////////////////////	 
//������EEPROM����ĵ�ַ�����ַ,ռ��13���ֽ�(RANGE:SAVE_ADDR_BASE~SAVE_ADDR_BASE+12)
#define SAVE_ADDR_BASE (uint32_t )0xA10000 //Flash ����У׼���ݵĴ�� ��ַ��Ҫ�������߻�Ӱ�쵽�����ֿ����ͼƬ
//����У׼����										    
void TP_Save_Adjdata(void)
{
	u8 tbuff[4];
	s32 temp;			 
	//����У�����!		   							  
	temp=tp_dev.xfac*100000000;//����xУ������  
	//�洢temp���ⲿflash��
  tbuff[0]=temp>>24;
	tbuff[1]=temp>>16;	
	tbuff[2]=temp>>8;	
	tbuff[3]=temp;	
  W25QXX_Write((u8*)tbuff,SAVE_ADDR_BASE,4);
  
	temp=tp_dev.yfac*100000000;//����yУ������  
  //�洢temp���ⲿflash��  
  tbuff[0]=temp>>24;
	tbuff[1]=temp>>16;	
	tbuff[2]=temp>>8;	
	tbuff[3]=temp;	
  W25QXX_Write((u8*)tbuff,SAVE_ADDR_BASE+4,4);
	//����xƫ����
	tbuff[0]=tp_dev.xoff>>8;	
	tbuff[1]=tp_dev.xoff;	
  W25QXX_Write((u8*)tbuff,SAVE_ADDR_BASE+8,2);	    
	//����yƫ����
	tbuff[0]=tp_dev.yoff>>8;	
	tbuff[1]=tp_dev.yoff;	
  W25QXX_Write((u8*)tbuff,SAVE_ADDR_BASE+10,2);
	//���津������
	tbuff[0]=tp_dev.touchtype;	
  W25QXX_Write((u8*)tbuff,SAVE_ADDR_BASE+12,1);
	
	tbuff[0]=0X0A;	//���У׼����
  W25QXX_Write((u8*)tbuff,SAVE_ADDR_BASE+13,1);
}
//�õ�������EEPROM�����У׼ֵ
//����ֵ��1���ɹ���ȡ����
//        0����ȡʧ�ܣ�Ҫ����У׼
u8 TP_Get_Adjdata(void)
{		
  u8 tbuff[4];	
	s32 tempfac;
  W25QXX_Read(tbuff,SAVE_ADDR_BASE+13,1);//��ȡ�����,���Ƿ�У׼���� 
  tempfac=tbuff[0];
	printf("tempfac=%x \r\n",tempfac);
	
	if(tempfac==0X0A)//�������Ѿ�У׼����			   
	{ 
    W25QXX_Read(tbuff,SAVE_ADDR_BASE,4);
    tempfac =tbuff[0]; tempfac<<=8;
		tempfac|=tbuff[1]; tempfac<<=8;
		tempfac|=tbuff[2]; tempfac<<=8;
		tempfac|=tbuff[3];		   
		tp_dev.xfac=(float)tempfac/100000000;//�õ�xУ׼����
		
		W25QXX_Read(tbuff,SAVE_ADDR_BASE+4,4);
    tempfac =tbuff[0]; tempfac<<=8;
		tempfac|=tbuff[1]; tempfac<<=8;
		tempfac|=tbuff[2]; tempfac<<=8;
		tempfac|=tbuff[3];			          
		tp_dev.yfac=(float)tempfac/100000000;//�õ�yУ׼����
	    //�õ�xƫ����
		
		W25QXX_Read(tbuff,SAVE_ADDR_BASE+8,2);
    tp_dev.xoff =tbuff[0]; tp_dev.xoff<<=8;
		tp_dev.xoff|=tbuff[1];  		   	  
 	    //�õ�yƫ����
		
		W25QXX_Read(tbuff,SAVE_ADDR_BASE+10,2);
    tp_dev.yoff =tbuff[0]; tp_dev.yoff<<=8;
		tp_dev.yoff|=tbuff[1];
			
    W25QXX_Read(tbuff,SAVE_ADDR_BASE+12,1);//��ȡ�������ͱ��
    tp_dev.touchtype =tbuff[0];
	
		if(tp_dev.touchtype)//X,Y��������Ļ�෴
		{
			CMD_RDX=0X90;
			CMD_RDY=0XD0;	 
		}else				   //X,Y��������Ļ��ͬ
		{
			CMD_RDX=0XD0;
			CMD_RDY=0X90;	 
		}		 
		return 1;	 
	}
	return 0;
}	 
//��ʾ�ַ���
u8* const TP_REMIND_MSG_TBL="Please use the stylus click the cross on the screen.The cross will always move until the screen adjustment is completed.";
 					  
//��ʾУ׼���(��������)
void TP_Adj_Info_Show(u16 x0,u16 y0,u16 x1,u16 y1,u16 x2,u16 y2,u16 x3,u16 y3,u16 fac,u16 color)
{	  
	POINT_COLOR=RED;
	LCD_ShowString(40,160,lcddev.width,lcddev.height,16,"x1:",color);
 	LCD_ShowString(40+80,160,lcddev.width,lcddev.height,16,"y1:",color);
 	LCD_ShowString(40,180,lcddev.width,lcddev.height,16,"x2:",color);
 	LCD_ShowString(40+80,180,lcddev.width,lcddev.height,16,"y2:",color);
	LCD_ShowString(40,200,lcddev.width,lcddev.height,16,"x3:",color);
 	LCD_ShowString(40+80,200,lcddev.width,lcddev.height,16,"y3:",color);
	LCD_ShowString(40,220,lcddev.width,lcddev.height,16,"x4:",color);
 	LCD_ShowString(40+80,220,lcddev.width,lcddev.height,16,"y4:",color);  
 	LCD_ShowString(40,240,lcddev.width,lcddev.height,16,"fac is:",color);     
	LCD_ShowNum(40+24,160,x0,4,16,color);		//��ʾ��ֵ
	LCD_ShowNum(40+24+80,160,y0,4,16,color);	//��ʾ��ֵ
	LCD_ShowNum(40+24,180,x1,4,16,color);		//��ʾ��ֵ
	LCD_ShowNum(40+24+80,180,y1,4,16,color);	//��ʾ��ֵ
	LCD_ShowNum(40+24,200,x2,4,16,color);		//��ʾ��ֵ
	LCD_ShowNum(40+24+80,200,y2,4,16,color);	//��ʾ��ֵ
	LCD_ShowNum(40+24,220,x3,4,16,color);		//��ʾ��ֵ
	LCD_ShowNum(40+24+80,220,y3,4,16,color);	//��ʾ��ֵ
 	LCD_ShowNum(40+56,240,fac,3,16,color); 	//��ʾ��ֵ,����ֵ������95~105��Χ֮��.

}
		 
//������У׼����
//�õ��ĸ�У׼����
void TP_Adjust(void)
{								 
	u16 pos_temp[4][2];//���껺��ֵ
	u8  cnt=0;	
	u16 d1,d2;
	u32 tem1,tem2;
	double fac; 	
	u16 outtime=0;
 	cnt=0;				
	POINT_COLOR=BLUE;
	BACK_COLOR =WHITE;
	LCD_Clear(WHITE);//����   
	POINT_COLOR=RED;//��ɫ 
	LCD_Clear(WHITE);//���� 	   
	POINT_COLOR=BLACK;
	LCD_ShowString(40,40,160,100,16,(u8*)TP_REMIND_MSG_TBL,POINT_COLOR);//��ʾ��ʾ��Ϣ
	TP_Drow_Touch_Point(20,20,RED);//����1 
	tp_dev.sta=0;//���������ź� 
	tp_dev.xfac=0;//xfac��������Ƿ�У׼��,����У׼֮ǰ�������!�������	 
	while(1)//�������10����û�а���,���Զ��˳�
	{
		tp_dev.scan(1);//ɨ����������
		if((tp_dev.sta&0xc0)==TP_CATH_PRES)//����������һ��(��ʱ�����ɿ���.)
		{	
			outtime=0;		
			tp_dev.sta&=~(1<<6);//��ǰ����Ѿ����������.
						   			   
			pos_temp[cnt][0]=tp_dev.x[0];
			pos_temp[cnt][1]=tp_dev.y[0];
			cnt++;	  
			switch(cnt)
			{			   
				case 1:						 
					TP_Drow_Touch_Point(20,20,WHITE);				//�����1 
					TP_Drow_Touch_Point(lcddev.width-20,20,RED);	//����2
					break;
				case 2:
 					TP_Drow_Touch_Point(lcddev.width-20,20,WHITE);	//�����2
					TP_Drow_Touch_Point(20,lcddev.height-20,RED);	//����3
					break;
				case 3:
 					TP_Drow_Touch_Point(20,lcddev.height-20,WHITE);			//�����3
 					TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,RED);	//����4
					break;
				case 4:	 //ȫ���ĸ����Ѿ��õ�
	    		    //�Ա����
					tem1=abs(pos_temp[0][0]-pos_temp[1][0]);//x1-x2
					tem2=abs(pos_temp[0][1]-pos_temp[1][1]);//y1-y2
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//�õ�1,2�ľ���
					
					tem1=abs(pos_temp[2][0]-pos_temp[3][0]);//x3-x4
					tem2=abs(pos_temp[2][1]-pos_temp[3][1]);//y3-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//�õ�3,4�ľ���
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05||d1==0||d2==0)//���ϸ�
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//�����4
   	 					TP_Drow_Touch_Point(20,20,RED);								//����1
 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100,RED);//��ʾ����   
 						continue;
					}
					tem1=abs(pos_temp[0][0]-pos_temp[2][0]);//x1-x3
					tem2=abs(pos_temp[0][1]-pos_temp[2][1]);//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//�õ�1,3�ľ���
					
					tem1=abs(pos_temp[1][0]-pos_temp[3][0]);//x2-x4
					tem2=abs(pos_temp[1][1]-pos_temp[3][1]);//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//�õ�2,4�ľ���
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)//���ϸ�
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//�����4
   	 					TP_Drow_Touch_Point(20,20,RED);								//����1
 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100,RED);//��ʾ����   
						continue;
					}//��ȷ��
								   
					//�Խ������
					tem1=abs(pos_temp[1][0]-pos_temp[2][0]);//x1-x3
					tem2=abs(pos_temp[1][1]-pos_temp[2][1]);//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//�õ�1,4�ľ���
	
					tem1=abs(pos_temp[0][0]-pos_temp[3][0]);//x2-x4
					tem2=abs(pos_temp[0][1]-pos_temp[3][1]);//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//�õ�2,3�ľ���
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)//���ϸ�
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//�����4
   	 					TP_Drow_Touch_Point(20,20,RED);								//����1
 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100,RED);//��ʾ����   
						continue;
					}//��ȷ��
					//������
					tp_dev.xfac=(float)(lcddev.width-40)/(pos_temp[1][0]-pos_temp[0][0]);//�õ�xfac		 
					tp_dev.xoff=(lcddev.width-tp_dev.xfac*(pos_temp[1][0]+pos_temp[0][0]))/2;//�õ�xoff
						  
					tp_dev.yfac=(float)(lcddev.height-40)/(pos_temp[2][1]-pos_temp[0][1]);//�õ�yfac
					tp_dev.yoff=(lcddev.height-tp_dev.yfac*(pos_temp[2][1]+pos_temp[0][1]))/2;//�õ�yoff  
					if(abs(tp_dev.xfac)>2||abs(tp_dev.yfac)>2)//������Ԥ����෴��.
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//�����4
   	 					TP_Drow_Touch_Point(20,20,RED);								//����1
						LCD_ShowString(40,26,lcddev.width,lcddev.height,16,"TP Need readjust!",POINT_COLOR);
						tp_dev.touchtype=!tp_dev.touchtype;//�޸Ĵ�������.
						if(tp_dev.touchtype)//X,Y��������Ļ�෴
						{
							CMD_RDX=0X90;
							CMD_RDY=0XD0;	 
						}else				   //X,Y��������Ļ��ͬ
						{
							CMD_RDX=0XD0;
							CMD_RDY=0X90;	 
						}			    
						continue;
					}		
					POINT_COLOR=BLUE;
					LCD_Clear(WHITE);//����
					LCD_ShowString(35,110,lcddev.width,lcddev.height,16,"Touch Screen Adjust OK!",POINT_COLOR);//У�����
					
					printf("tp_dev.xfac:%f\r\n",tp_dev.xfac);
					printf("tp_dev.yfac:%f\r\n",tp_dev.yfac);
					printf("tp_dev.xoff:%d\r\n",tp_dev.xoff);
					printf("tp_dev.yoff:%d\r\n",tp_dev.yoff);
					
					delay_ms(1000);
					TP_Save_Adjdata();  
 					LCD_Clear(WHITE);//����   
					return;//У�����				 
			}
		}
		delay_ms(10);
		outtime++;
		if(outtime>1000)
		{
			TP_Get_Adjdata();
			break;
	 	} 
 	}
}	 
//��������ʼ��  		    
//����ֵ:1,û�н���У׼
//       1,���й�У׼
u8 TP_Init(void)
{	
	if(CST3XX_Init()==0)
	{
		tp_dev.scan=CST3XX_Scan;  
		tp_dev.touchtype |= 0X80;       /* ��ǵ����� */
		return 1;
	}
	else if(GT911_Init()==0)
	{
		tp_dev.scan=GT911_Scan;       	//ɨ�躯��ָ��GT911������ɨ��
		tp_dev.touchtype |= 0X80;       /* ��ǵ����� */
		return 1;
	}else
	{ 
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC,ENABLE);//ʹ��PORTA C
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;// PC1�˿�����
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//�������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		GPIO_SetBits(GPIOC,GPIO_Pin_14);//���1
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;// PA0/1/2 �˿�����
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//�������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_SetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2);//���1
		
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;			  //PA1�˿�����
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;		//��������
		GPIO_Init(GPIOA, &GPIO_InitStructure);			  //PA1��������
		GPIO_ResetBits(GPIOA,GPIO_Pin_1);				      //����
		

		TP_Read_XY(&tp_dev.x[0],&tp_dev.y[0]);//��һ�ζ�ȡ��ʼ��	 
		//printf("xtemp:%d\r\n",x0temp);	
	  //printf("ytemp:%d\r\n",y0temp);	
	  if(x0temp==0&&y0temp==0)
    {
		   AUTO=1;
			 tp_dev.scan=Null_Scan; 
			 return 1;	 
		 }//�����ڵ��败��оƬ���Զ�ˢͼ
		
		W25QXX_Init();//��ʼ��25QXX
		if(TP_Get_Adjdata())return 1;//�Ѿ�У׼
		else			        //δУ׼?
		{ 
			LCD_Clear(WHITE);//����
			TP_Adjust();  	 //��ĻУ׼ 
			TP_Save_Adjdata();	 
		}			
		TP_Get_Adjdata();	
		
		return 0; 									 
	}
}

//�жϴ����ǲ�����ָ������֮��
//(x,y):��ʼ����
//xlen,ylen:��x,y�����ϵ�ƫ�Ƴ���
//����ֵ :1,�ڸ�������.0,���ڸ�������.
u8 Is_In_Area(u16 x,u16 y,u16 xlen,u16 ylen)
{
	tp_dev.scan(0);//ɨ��
	if(tp_dev.sta&TP_PRES_DOWN)//�а���������
		{
			delay_ms(20);//��Ҫ����ʱ,��������Ϊ�а�������.
			if(tp_dev.x[0]<=(x+xlen)&&tp_dev.x[0]>=x&&tp_dev.y[0]<=(y+ylen)&&tp_dev.y[0]>=y)
			{
				return 1;
			}
			else return 0;
		}
	 return 0;
} 


//�����ַ����
u16 PositionH[4*2]={
0,LCD_WIDTH-40,40,40,     //��һ������λ�ã� ��ʼλ��X����ʼλ��Y����ȣ�����
LCD_HEIGHT-40,LCD_WIDTH-40,40,40,    //�ڶ�������λ�ã� ��ʼλ��X����ʼλ��Y����ȣ�����
};
u16 PositionV[4*2]={
0,LCD_HEIGHT-40,40,40,    //��һ������λ�ã� ��ʼλ��X����ʼλ��Y����ȣ�����
LCD_WIDTH-40,LCD_HEIGHT-40,40,40,    //�ڶ�������λ�ã� ��ʼλ��X����ʼλ��Y����ȣ�����
};

u16 touchcount=0;
u8 lock=0;
//����λ��ɨ��
u8 py_get_keynum(void)
{
	u8 KeyNO;
	u16 X,Y,WIDTH,HEIGHT;
	u8 i;
	static u8 key_x=0;//0,û���κΰ�������;1~9,1~9�Ű�������
	u8 key=0;
	u16 *Pos;//��������
	tp_dev.scan(0); 		 
	if(tp_dev.sta&TP_PRES_DOWN)			//������������
	{	
		 if(lcddev.dir==0) Pos = PositionV; //����
		 else              Pos = PositionH; //���� 
		 KeyNO=2;//ֻ����������
		//ɨ�谴��λ��
		for (i = 0; i < KeyNO; i++)
		{
				X = Pos[i*4 + 0];
				Y = Pos[i*4 + 1];
			  WIDTH = Pos[i*4 + 2]; 
			  HEIGHT = Pos[i*4 + 3]; //��ȡ��������
			
				if(tp_dev.x[0]<(X+WIDTH)&&tp_dev.x[0]>(X)&&tp_dev.y[0]<(Y+HEIGHT)&&tp_dev.y[0]>(Y)) { key=i+1; break;}//�ҵ�����λ�ã�����ѭ��
		 }	
			if(key) //���ֵ����ȷ��
			{	  
          if((touchcount++>500)&&(key_x!=0))//������	
					{
						key_x=key;
					}else//�̰�
          {					
						if(key_x==key)key=0;
						else          key_x=key;
					}
			}
		}else if(key_x) //���� ����LOCK��Ϊ�жϵ�������꣬��ֹ�ƶ��ı�����
	  {
			  touchcount=0;
		    key_x=0;
		    lock=0;//�ͷŰ�����
 
	  } 
	 return key;
}


//�����Ļ�������Ͻ���ʾ"RST"
void Load_Drow_Dialog(void)
{
	LCD_Clear(WHITE);	//����   
	LCD_ShowString(lcddev.width-24,0,200,16,16,"RST",BLUE);//��ʾ��������
	Show_Str(0,lcddev.height-20,50,16,"����",16,1,BLUE);
}

//��ʾ���غͼ���
void ShowUI(u8 i)
{
	if(i>1) Show_Str(0,lcddev.height-20,50,16,"����",16,1,POINT_COLOR); //���ڵ�һ���������Ҫ����
	        Show_Str(lcddev.width-40,lcddev.height-20,50,16,"����",16,1,POINT_COLOR);
}

////////////////////////////////////////////////////////////////////////////////
//���ݴ�����ר�в���
//��ˮƽ��
//x0,y0:����
//len:�߳���
//color:��ɫ
void gui_draw_hline(u16 x0,u16 y0,u16 len,u16 color)
{
	if(len==0)return;
	LCD_Fill(x0,y0,x0+len-1,y0,color);	
}
//��ʵ��Բ
//x0,y0:����
//r:�뾶
//color:��ɫ
void gui_fill_circle(u16 x0,u16 y0,u16 r,u16 color)
{											  
	u32 i;
	u32 imax = ((u32)r*707)/1000+1;
	u32 sqmax = (u32)r*(u32)r+(u32)r/2;
	u32 x=r;
	gui_draw_hline(x0-r,y0,2*r,color);
	for (i=1;i<=imax;i++) 
	{
		if ((i*i+x*x)>sqmax)// draw lines from outside  
		{
 			if (x>imax) 
			{
				gui_draw_hline (x0-i+1,y0+x,2*(i-1),color);
				gui_draw_hline (x0-i+1,y0-x,2*(i-1),color);
			}
			x--;
		}
		// draw lines from inside (center)  
		gui_draw_hline(x0-x,y0+i,2*x,color);
		gui_draw_hline(x0-x,y0-i,2*x,color);
	}
}  
//������֮��ľ���ֵ 
//x1,x2����ȡ��ֵ��������
//����ֵ��|x1-x2|
u16 my_abs(u16 x1,u16 x2)
{			 
	if(x1>x2)return x1-x2;
	else return x2-x1;
}  
//��һ������
//(x1,y1),(x2,y2):��������ʼ����
//size�������Ĵ�ϸ�̶�
//color����������ɫ
void lcd_draw_bline(u16 x1, u16 y1, u16 x2, u16 y2,u8 size,u16 color)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 
	if(x1<size|| x2<size||y1<size|| y2<size)return; 
	delta_x=x2-x1; //������������ 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; //���õ������� 
	else if(delta_x==0)incx=0;//��ֱ�� 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;//ˮƽ�� 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; //ѡȡ�������������� 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )//������� 
	{  
		gui_fill_circle(uRow,uCol,size,color);//���� 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
} 
//5�����ص����ɫ												 
const u16 POINT_COLOR_TBL[CT_MAX_TOUCH]={RED,GREEN,BLUE,BROWN,GRED};  
//���ݴ��������Ժ���
void ctp_test(void)
{
	u8 t=0;
	u8 i=0;	  	    
 	u16 lastpos[5][2];		//��¼���һ�ε����� 
	lastpos[0][0]=0;
	lastpos[0][1]=0;
	while(1)
	{
		tp_dev.scan(0);    //��ɨ�躯��ȡX Y���� 
		for(t=0;t<CT_MAX_TOUCH;t++)
		{
			if((tp_dev.sta)&(1<<t))
			{
				if(tp_dev.x[t]<lcddev.width&&tp_dev.y[t]<lcddev.height)
				{
					printf("x:%d\r\n",tp_dev.x[0]);	
					printf("y:%d\r\n",tp_dev.y[0]);	 
					
					if(lastpos[t][0]==0XFFFF)
					{
						lastpos[t][0] = tp_dev.x[t];
						lastpos[t][1] = tp_dev.y[t];
					}
					lcd_draw_bline(lastpos[t][0],lastpos[t][1],tp_dev.x[t],tp_dev.y[t],2,POINT_COLOR_TBL[t]);//����
					lastpos[t][0]=tp_dev.x[t];
					lastpos[t][1]=tp_dev.y[t];
					if(tp_dev.x[t]>(lcddev.width-40)&&tp_dev.y[t]<40)
					{
						Load_Drow_Dialog();//���
					}else if(tp_dev.x[t]<(0+24)&&tp_dev.x[t]>(0)&&tp_dev.y[t]<(lcddev.height)&&tp_dev.y[t]>(lcddev.height-24))//���½�
					{
						LCD_Clear(WHITE);
						ShowUI(1);
						return; //����
					}
				}
			}else lastpos[t][0]=0XFFFF;	
		}
		delay_ms(5);i++;
	}	
}

//���败�������Ժ���
void rtp_test(void)
{
	while(1)
	{
		tp_dev.scan(0); 		 
		if(tp_dev.sta&TP_PRES_DOWN)			//������������
		{	
		 	if(tp_dev.x[0]<lcddev.width&&tp_dev.y[0]<lcddev.height)
			{	
				if(tp_dev.x[0]>(lcddev.width-24)&&tp_dev.y[0]<16)Load_Drow_Dialog();//���
				else if(tp_dev.x[0]<(0+24)&&tp_dev.x[0]>(0)&&tp_dev.y[0]<(lcddev.height)&&tp_dev.y[0]>(lcddev.height-24))//���½�
				{
						LCD_Clear(WHITE);
						ShowUI(1);
						return; //����
				}
				else TP_Draw_Big_Point(tp_dev.x[0],tp_dev.y[0],RED);		//��ͼ	  			   
			}
		}else delay_ms(10);	//û�а������µ�ʱ�� 	    
	}
}
