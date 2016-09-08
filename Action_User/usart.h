#include "stm32f4xx_usart.h"


//PB6 7 UART1
/* Definition for USARTx resources ********************************************/
#define USARTx_CLK                       RCC_APB2Periph_USART1
#define USARTx_CLK_INIT                  RCC_APB2PeriphClockCmd
//#define USARTx_IRQn                      USART3_IRQn
//#define USARTx_IRQHandler                USART3_IRQHandler

#define USARTx_TX_PIN                    GPIO_Pin_6                
#define USARTx_TX_GPIO_PORT              GPIOB                       
#define USARTx_TX_GPIO_CLK               RCC_AHB1Periph_GPIOB
#define USARTx_TX_SOURCE                 GPIO_PinSource6
#define USARTx_TX_AF                     GPIO_AF_USART1

#define USARTx_RX_PIN                    GPIO_Pin_7                
#define USARTx_RX_GPIO_PORT              GPIOB                    
#define USARTx_RX_GPIO_CLK               RCC_AHB1Periph_GPIOB
#define USARTx_RX_SOURCE                 GPIO_PinSource7
#define USARTx_RX_AF                     GPIO_AF_USART1

/* Definition for DMAx resources **********************************************/
//#define USARTx_DR_ADDRESS                ((uint32_t)USART3 + 0x04) 

#define USARTx_DMA                       DMA2
#define USARTx_DMAx_CLK                  RCC_AHB1Periph_DMA2
	 
#define USARTx_TX_DMA_CHANNEL            DMA_Channel_4
#define USARTx_TX_DMA_STREAM             DMA2_Stream7
#define USARTx_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF3
#define USARTx_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF3
#define USARTx_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF3
#define USARTx_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF3
#define USARTx_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF3
						
//#define USARTx_RX_DMA_CHANNEL            DMA_Channel_4
//#define USARTx_RX_DMA_STREAM             DMA1_Stream1
//#define USARTx_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF1
//#define USARTx_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF1
//#define USARTx_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF1
//#define USARTx_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF1
//#define USARTx_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF1

//#define USARTx_DMA_TX_IRQn               DMA2_Stream7_IRQn
//#define USARTx_DMA_RX_IRQn               DMA1_Stream1_IRQn
//#define USARTx_DMA_TX_IRQHandler         DMA1_Stream3_IRQHandler
//#define USARTx_DMA_RX_IRQHandler         DMA1_Stream1_IRQHandler

void DMA_USART_Config(void);
void USART_DMA_Send(char *Data2Send, uint32_t DataSize);

void ROBS_USART1_Init(uint32_t BaudRate);
void BLE_UART5_Init(uint32_t BaudRate);
void GYRO_USART3_Init(uint32_t BaudRate);
void UART1_SendString (unsigned char *s);
void UART1_SendChar(int32_t disp);
void UART2_SendString (unsigned char *s);
void UART2_SendChar(int32_t disp);
void UART3_SendString (unsigned char *s);
void UART3_SendChar(int32_t disp);
void UART3SendGyro(double ddata);
uint16_t GetUSART2_ReceiveData(void);
 void SetUSART2_ReceiveData(uint16_t data);
 void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...);
char *itoa(int value, char *string, int radix);

void HandleDecode(void);
int32_t IsHandleReceive(void);
void SetHandleRxState(void);
void ClearHandleRxState(void);
void DJHandle(void);
void UART1double(double ddata);
void Usart_Send_XY(void);
void DEBUG_printf(void);

