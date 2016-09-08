#include "elmo.h"
#include "can.h"

 /**************初始化驱动器********************/
void elmo_Init()
{
	uint32_t data[1][2]={0x00000001,00000000};
	CAN_TxMsg(CAN1,0x000,(uint8_t*)&data[0],8);
}
/****************使能电机***************************/
void elmo_Enable(uint8_t ElmoNum)
{
	 uint32_t data[1][2]={    				 

							0x00004F4D,0x00000001,      //MO  1
						 };
  uint8_t mbox;	 
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300 + ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300 + ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
    
	    //msg[4].data=*(unsigned long long*)&data[i];	  	
	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;
		
	mbox= CAN_Transmit(CAN1, &TxMessage);         //1.4us	
	while((CAN_TransmitStatus(CAN1, mbox)!= CAN_TxStatus_Ok));//等待238us
}	
/**************失能电机***************************/
void elmo_Disable(uint8_t ElmoNum)
{
	 uint8_t i=0; 
	 uint32_t data[1][2]={    				 

//							0x0000564A,0x00000000,		//JV  10000
//							0x40004742,0x00000000,    //BG 
							0x00004F4D,0x00000000,      //MO  0
						 };
  uint8_t mbox;	 
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300+ ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300+ ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
  for(i=0;i<1;i++)
	{	   

		TxMessage.Data[0] = *(unsigned long*)&data[i][0]&0xff;
		TxMessage.Data[1] = (*(unsigned long*)&data[i][0]>>8)&0xff;
		TxMessage.Data[2] = (*(unsigned long*)&data[i][0]>>16)&0xff;
		TxMessage.Data[3] = (*(unsigned long*)&data[i][0]>>24)&0xff;
		TxMessage.Data[4] = *(unsigned long*)&data[i][1]&0xff;
		TxMessage.Data[5] = (*(unsigned long*)&data[i][1]>>8)&0xff;
		TxMessage.Data[6] = (*(unsigned long*)&data[i][1]>>16)&0xff;
		TxMessage.Data[7] = (*(unsigned long*)&data[i][1]>>24)&0xff;
		mbox= CAN_Transmit(CAN1, &TxMessage);         //1.4us	
	  while((CAN_TransmitStatus(CAN1, mbox)!= CAN_TxStatus_Ok));//等待238us
	}	

}	
/***************速度环配置*************************/
void	Vel_cfg(uint8_t ElmoNum,uint32_t acc,uint32_t dec)
{
	 uint8_t i=0; 
	 uint32_t data[7][2]={    				 
							0x00004D55,0x00000002,      //UM  2	
							0x00004653,0x00000000,		//SF  0
							0x00004F4D,0x00000001,      //MO  1
							0x00004341,0x00000000,		//AC  3000000
							0x00004344,0x00000000,		//DC  3000000
							0x00024856,0x00000000,		//VH[2]  15,000,000
							0x00024C56,0x00000000,		//VL[2]  -15,000,000
						 };
	 

  uint8_t mbox;	 
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300+ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300+ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
	data[3][1]= acc;
	data[4][1]= dec;
  data[5][1]=	 1000000000;
	data[6][1]=	-1000000000;	
	//data[7][1]= vel;							 
	for(i=0;i<7;i++)
	{	    
 	
			TxMessage.Data[0] = *(unsigned long*)&data[i][0]&0xff;
			TxMessage.Data[1] = (*(unsigned long*)&data[i][0]>>8)&0xff;
			TxMessage.Data[2] = (*(unsigned long*)&data[i][0]>>16)&0xff;
			TxMessage.Data[3] = (*(unsigned long*)&data[i][0]>>24)&0xff;
			TxMessage.Data[4] = *(unsigned long*)&data[i][1]&0xff;
			TxMessage.Data[5] = (*(unsigned long*)&data[i][1]>>8)&0xff;
			TxMessage.Data[6] = (*(unsigned long*)&data[i][1]>>16)&0xff;
			TxMessage.Data[7] = (*(unsigned long*)&data[i][1]>>24)&0xff;
				
			mbox= CAN_Transmit(CAN1, &TxMessage);         //1.4us	
			while((CAN_TransmitStatus(CAN1, mbox)!= CAN_TxStatus_Ok));//等待238us
	}
}

/************速度控制***************/
void VelCrl(uint8_t ElmoNum,int vel)
{
	 uint8_t i=0; 
	 uint32_t data[2][2]={    				 

							0x0000564A,0x00000000,		//JV  10000
							0x40004742,0x00000000,    //BG 
						 };
  uint8_t mbox;	 
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300+ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300+ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
	data[0][1]= vel;							 
	for(i=0;i<2;i++)
	{	    
	    //msg[4].data=*(unsigned long long*)&data[i];	  	
			TxMessage.Data[0] = *(unsigned long*)&data[i][0]&0xff;
			TxMessage.Data[1] = (*(unsigned long*)&data[i][0]>>8)&0xff;
			TxMessage.Data[2] = (*(unsigned long*)&data[i][0]>>16)&0xff;
			TxMessage.Data[3] = (*(unsigned long*)&data[i][0]>>24)&0xff;
			TxMessage.Data[4] = *(unsigned long*)&data[i][1]&0xff;
			TxMessage.Data[5] = (*(unsigned long*)&data[i][1]>>8)&0xff;
			TxMessage.Data[6] = (*(unsigned long*)&data[i][1]>>16)&0xff;
			TxMessage.Data[7] = (*(unsigned long*)&data[i][1]>>24)&0xff;
				
			mbox= CAN_Transmit(CAN1, &TxMessage);         //1.4us	
			while((CAN_TransmitStatus(CAN1, mbox)!= CAN_TxStatus_Ok));//等待238us
	}
}

/***************位置环配置*************************/
void Pos_cfg(uint8_t ElmoNum,uint32_t acc,uint32_t dec,uint32_t vel)
{
	uint8_t i=0; 
	uint32_t data[16][2]={  								 
							0x00004D55,0x00000005,    //UM  5
							0x00004653,0x00000000,		//SF  0
							0x00014D58,0x00000000,    //XM[1]   -5x10
							0x00024D58,0x00000000,    //XM[2]   5x10		
							0x00004F4D,0x00000001,    //MO  1
							0x00004341,0x00000000,		//AC
							0x00004341,0x00000000,		//AC
							0x00004341,0x00000000,		//AC		
							0x00004344,0x00000000,		//DC
							0x00004344,0x00000000,		//DC 
							0x00004344,0x00000000,		//DC 		
							0x00005053,0x00000000,		//SP
							0x00005053,0x00000000,		//SP
							0x00005053,0x00000000,		//SP		
							0x00034856,0x00000000,		//VH[3]  999,999,990
							0x00034C56,0x00000000,		//VL[3]  -99,999,990		
							
						 };

  uint8_t mbox;	 
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300+ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300+ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
		
	data[5][1]= acc;
	data[6][1]= acc;
	data[7][1]= acc;
	data[8][1]= dec;
  data[9][1]= dec;
	data[10][1]= dec;
	data[11][1]= vel;
	data[12][1]= vel;
	data[13][1]= vel;						 
	data[14][1]= 999999990;
	data[15][1]= -999999990;
	data[2][1]= -999999990;
	data[3][1]= 999999990;							 			 						 
  for(i=0;i<16;i++)
	{	    
		//msg[4].data=*(unsigned long long*)&data[i];	  	
		TxMessage.Data[0] = *(unsigned long*)&data[i][0]&0xff;
		TxMessage.Data[1] = (*(unsigned long*)&data[i][0]>>8)&0xff;
		TxMessage.Data[2] = (*(unsigned long*)&data[i][0]>>16)&0xff;
		TxMessage.Data[3] = (*(unsigned long*)&data[i][0]>>24)&0xff;
		TxMessage.Data[4] = *(unsigned long*)&data[i][1]&0xff;
		TxMessage.Data[5] = (*(unsigned long*)&data[i][1]>>8)&0xff;
		TxMessage.Data[6] = (*(unsigned long*)&data[i][1]>>16)&0xff;
		TxMessage.Data[7] = (*(unsigned long*)&data[i][1]>>24)&0xff;
			
		mbox= CAN_Transmit(CAN1, &TxMessage);         //1.4us	
		while((CAN_TransmitStatus(CAN1, mbox)!= CAN_TxStatus_Ok));//等待238us
	}
}

/*****************位置控制*********************/
/*******************1代表相对模式****************************/
/*******************0代表绝对模式****************************/
void PosCrl(uint8_t CopleyNum,uint8_t rel_abs,int pos)
{
	uint8_t i=0; 
	uint32_t data[2][2]={  								 
							0x00000000,0x00000000,      //PA  10000	
							0x40004742,0x00000000,      //BG 
						 };

  uint8_t mbox;	 
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300+CopleyNum;					 // standard identifier=0
	TxMessage.ExtId=0x300+CopleyNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;


	if(rel_abs==0)
	{
		data[0][0]= 0x00004150;  //绝对

	}
	else
	{
		data[0][0]= 0x00005250;   //相对

	}						
					 
	data[0][1]= pos;			 						 
  for(i=0;i<2;i++)
	{	    
		//msg[4].data=*(unsigned long long*)&data[i];	  	
		TxMessage.Data[0] = *(unsigned long*)&data[i][0]&0xff;
		TxMessage.Data[1] = (*(unsigned long*)&data[i][0]>>8)&0xff;
		TxMessage.Data[2] = (*(unsigned long*)&data[i][0]>>16)&0xff;
		TxMessage.Data[3] = (*(unsigned long*)&data[i][0]>>24)&0xff;
		TxMessage.Data[4] = *(unsigned long*)&data[i][1]&0xff;
		TxMessage.Data[5] = (*(unsigned long*)&data[i][1]>>8)&0xff;
		TxMessage.Data[6] = (*(unsigned long*)&data[i][1]>>16)&0xff;
		TxMessage.Data[7] = (*(unsigned long*)&data[i][1]>>24)&0xff;
			
		mbox= CAN_Transmit(CAN1, &TxMessage);         //1.4us	
		while((CAN_TransmitStatus(CAN1, mbox)!= CAN_TxStatus_Ok));//等待238us
	}

}

/* 读取电机电流 */
void ReadActualCurrent(uint8_t ElmoNum)
 {
	 uint32_t data[1][2]={    				 
							0x40005149,0x00000000,      //IQ  
						 };
  uint8_t mbox;	 
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300 + ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300 + ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
    
	    //msg[4].data=*(unsigned long long*)&data[i];	  	
	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;
		
	mbox= CAN_Transmit(CAN1, &TxMessage);         //1.4us	
	while((CAN_TransmitStatus(CAN1, mbox)!= CAN_TxStatus_Ok));//等待238us
 }
 /* 读取电机位置 */
void ReadActualPos(uint8_t ElmoNum)
 {
	 uint32_t data[1][2]={    				 
							0x40005850,0x00000000,      //PX
						 };
  uint8_t mbox;	 
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300 + ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300 + ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
    
	    //msg[4].data=*(unsigned long long*)&data[i];	  	
	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;
		
	mbox= CAN_Transmit(CAN1, &TxMessage);         //1.4us	
	while((CAN_TransmitStatus(CAN1, mbox)!= CAN_TxStatus_Ok));//等待238us
 }

/* 读取电机速度 */
void ReadActualVel(uint8_t ElmoNum)
{
	 uint32_t data[1][2]={    				 
							0x40005856,0x00000000,      //VX
						 };
  uint8_t mbox;	 
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300 + ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300 + ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
    
	    //msg[4].data=*(unsigned long long*)&data[i];	  	
	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;
		
	mbox= CAN_Transmit(CAN1, &TxMessage);         //1.4us	
	while((CAN_TransmitStatus(CAN1, mbox)!= CAN_TxStatus_Ok));//等待238us
}

