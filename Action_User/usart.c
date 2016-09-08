#include "usart.h"
#include "math.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stdarg.h"
#include "stm32f4xx_dma.h"
#include  <includes.h>
#include "stdio.h"

#define LENGTH 3


/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */


int fputc(int ch, FILE *f)
{
	 USART_SendData(UART5, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET)
  {}

  return ch;
}



//PC12:  UART5 Tx
//PD2 :  UART5 Rx
void BLE_UART5_Init(uint32_t BaudRate)
{
	USART_InitTypeDef USART_InitStructure;  
	//NVIC_InitTypeDef  NVIC_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	USART_InitStructure.USART_BaudRate = BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//------------------------------------------------------------
	 
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

  /* Connect PXx to USARTx_Tx*/
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
  /* Connect PXx to USARTx_Rx*/
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5);

  /* Configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure USART Rx as alternate function  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(UART5, & USART_InitStructure);
 
	/* Enable USART */
	USART_Cmd(UART5, ENABLE);

}


//PB6 USART1 Tx 
void ROBS_USART1_Init(uint32_t BaudRate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1); //GPIOB6复用为USART1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1); //GPIOB6复用为USART1
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; //GPIOB6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PB6

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = BaudRate;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器



}

//PC10  PC11 USART3  
void GYRO_USART3_Init(uint32_t BaudRate)
{
	USART_InitTypeDef USART_InitStructure;  
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	
	
	/* USARTx configured as follow:
	- BaudRate = 57600 baud  
	- Word Length = 8 Bits
	- One Stop Bit
	- No parity
	- Hardware flow control disabled (RTS and CTS signals)
	- Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//------------------------------------------------------------
	 
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

  /* Connect PXx to USARTx_Tx*/
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10,  GPIO_AF_USART3);
  /* Connect PXx to USARTx_Rx*/
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11,  GPIO_AF_USART3);

  /* Configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure USART Rx as alternate function  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(USART3, & USART_InitStructure);
  
	//////////   设置USART3中断       ///////////////
	NVIC_InitStructure.NVIC_IRQChannel=USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;

	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
	/* Enable USART */
	USART_Cmd(USART3, ENABLE);
 //------------------------------------------------------------
	//使能USART5接收中断,
 

}



void UART1_SendString (unsigned char *s) 
{
  while (*s != 0)
	{
   	 USART_SendData(USART1, *s++);
		 
	}
}
void UART1_SendChar(int32_t disp)
{

	uint16_t dispbuf[4];
	uint8_t i;
  if(disp>=0)
	{
	dispbuf[3] = disp%10 + '0';
	dispbuf[2] = disp/10%10 + '0';
	dispbuf[1] = disp/10/10%10 + '0';
	dispbuf[0] = disp/10/10/10%10 + '0';
	for(i=0;i<4;i++)
		 USART_SendData(USART1,dispbuf[i]);
	}
	else
	{
	dispbuf[3] = -disp%10 + '0';
	dispbuf[2] = -disp/10%10 + '0';
	dispbuf[1] = -disp/10/10%10 + '0';
	dispbuf[0] = -disp/10/10/10%10 + '0';
	USART_SendData(USART1,'-');
	for(i=0;i<4;i++)
		 USART_SendData(USART1,dispbuf[i]);
	}
}



void UART2_SendString (unsigned char *s) 
{
  while (*s != 0)
	{
   	 USART_SendData(USART2, *s++);
		 
	}
}
void UART2_SendChar(int32_t disp)
{

	uint16_t dispbuf[4];
	uint8_t i;
  if(disp>=0)
	{
	dispbuf[3] = disp%10 + '0';
	dispbuf[2] = disp/10%10 + '0';
	dispbuf[1] = disp/10/10%10 + '0';
	dispbuf[0] = disp/10/10/10%10 + '0';
	for(i=0;i<4;i++)
		 USART_SendData(USART2,dispbuf[i]);
	}
	else
	{
	dispbuf[3] = -disp%10 + '0';
	dispbuf[2] = -disp/10%10 + '0';
	dispbuf[1] = -disp/10/10%10 + '0';
	dispbuf[0] = -disp/10/10/10%10 + '0';
	USART_SendData(USART2,'-');
	for(i=0;i<4;i++)
		 USART_SendData(USART2,dispbuf[i]);
	}
}





void UART3_SendString (unsigned char *s) 
{
  while (*s != 0)
	{
   	 USART_SendData(USART3, *s++);
	}
}
void UART3_SendChar(int32_t disp)
{

	uint16_t dispbuf[6];
	uint8_t i;
  if(disp>=0)
	{
	dispbuf[5] = disp%10 + '0';
	dispbuf[4] = disp/10%10 + '0';
	dispbuf[3] = disp/10/10%10 + '0';
	dispbuf[2] = disp/10/10/10%10 + '0';
	dispbuf[1] = disp/10/10/10/10%10 + '0';
	dispbuf[0] = disp/10/10/10/10/10%10 + '0';
	for(i=0;i<5;i++)
		 USART_SendData(USART3,dispbuf[i]);
	}
	else
	{
	dispbuf[5] = -disp%10 + '0';
	dispbuf[4] = -disp/10%10 + '0';
	dispbuf[3] = -disp/10/10%10 + '0';
	dispbuf[2] = -disp/10/10/10%10 + '0';
	dispbuf[1] = -disp/10/10/10/10%10 + '0';
	dispbuf[0] = -disp/10/10/10/10/10%10 + '0';
	
	USART_SendData(USART3,'-');
	for(i=0;i<5;i++)
		 USART_SendData(USART3,dispbuf[i]);
	}
}


void UART3SendGyro(double ddata)
{
	unsigned char i;
    char buf[3+LENGTH];


	if(ddata>0)
	{
		ddata=ddata*pow(10,LENGTH);
		for(i=0;i<(3+LENGTH);i++)
			{
			 	buf[2+LENGTH-i]=((long int)ddata/(signed int)pow(10,i))%10+'0';
			}
		for(i=0;i<(3+LENGTH);i++)
		{
			USART_SendData(USART3,buf[i]);
			if(i==2)
				USART_SendData(USART3,'.');
		}
	}

	else
	{
		ddata=(-ddata)*pow(10,LENGTH);
		for(i=0;i<(3+LENGTH);i++)
			{
			 	buf[2+LENGTH-i]=((long int)ddata/(signed int)pow(10,i))%10+'0';
			}
		USART_SendData(USART3,'-');
		for(i=0;i<(3+LENGTH);i++)
		{
			USART_SendData(USART3,buf[i]);
			if(i==2)
				USART_SendData(USART3,'.');
		}
	}


}

void UART1double(double ddata)
{
	unsigned char i;
    char buf[3+LENGTH];


	if(ddata>0)
	{
		ddata=ddata*pow(10,LENGTH);
		for(i=0;i<(3+LENGTH);i++)
			{
			 	buf[2+LENGTH-i]=((long int)ddata/(signed int)pow(10,i))%10+'0';
			}
		for(i=0;i<(3+LENGTH);i++)
		{
			USART_SendData(USART1,buf[i]);
			if(i==2)
				USART_SendData(USART1,'.');
		}
	}

	else
	{
		ddata=(-ddata)*pow(10,LENGTH);
		for(i=0;i<(3+LENGTH);i++)
			{
			 	buf[2+LENGTH-i]=((long int)ddata/(signed int)pow(10,i))%10+'0';
			}
		USART_SendData(USART1,'-');
		for(i=0;i<(3+LENGTH);i++)
		{
			USART_SendData(USART1,buf[i]);
			if(i==2)
				USART_SendData(USART1,'.');
		}
	}


}


static uint16_t Uart2Data=0;
 uint16_t GetUSART2_ReceiveData(void)
 {
	 return Uart2Data;
 }
 void SetUSART2_ReceiveData(uint16_t data)
 {
	 Uart2Data=data;
 }
 
 
 
 
 /****************************************************************************
* 名    称：void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...)
* 功    能：格式化串口输出函数
* 入口参数：USARTx:  指定串口
			Data：   发送数组
			...:     不定参数
* 出口参数：无
* 说    明：格式化串口输出函数
        	"\r"	回车符	   USART_OUT(USART1, "abcdefg\r")   
			"\n"	换行符	   USART_OUT(USART1, "abcdefg\r\n")
			"%s"	字符串	   USART_OUT(USART1, "字符串是：%s","abcdefg")
			"%d"	十进制	   USART_OUT(USART1, "a=%d",10)
* 调用方法：无 
****************************************************************************/
void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...){ 
	const char *s;
    int d;
    char buf[16];
    va_list ap;
    va_start(ap, Data);

	while(*Data!=0){				                          //判断是否到达字符串结束符
		if(*Data==0x5c){									  //'\'
			switch (*++Data){
				case 'r':							          //回车符
					USART_SendData(USARTx, 0x0d);	   

					Data++;
					break;
				case 'n':							          //换行符
					USART_SendData(USARTx, 0x0a);	
					Data++;
					break;
				
				default:
					Data++;
				    break;
			}
			
			 
		}
		else if(*Data=='%'){									  //
			switch (*++Data){				
				case 's':										  //字符串
                	s = va_arg(ap, const char *);
                	for ( ; *s; s++) {
                    	USART_SendData(USARTx,*s);
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                	}
					Data++;
                	break;
            	case 'd':										  //十进制
                	d = va_arg(ap, int);
                	itoa(d, buf, 10);
                	for (s = buf; *s; s++) {
                    	USART_SendData(USARTx,*s);
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                	}
					Data++;
                	break;
				default:
					Data++;
				    break;
			}		 
		}
		else USART_SendData(USARTx, *Data++);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
	}
}

/******************************************************
		整形数据转字符串函数
        char *itoa(int value, char *string, int radix)
		radix=10 标示是10进制	非十进制，转换结果为0;  

	    例：d=-379;
		执行	itoa(d, buf, 10); 后
		
		buf="-379"							   			  
**********************************************************/
char *itoa(int value, char *string, int radix)
{
    int     i, d;
    int     flag = 0;
    char    *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';

        /* Make the value positive. */
        value *= -1;
    }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;

} 
