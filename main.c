#include "Nano100Series.h"
#include <stdio.h>

//------------------ global variable ------------------
int i,j,r;
char b[32];
uint32_t REDans;					// 660nm
uint32_t newREDans=0;
uint32_t oldREDans=0;
//uint32_t DCnewREDans=0;
//uint32_t DColdREDans=0;
uint32_t IRans;						// 880nm
uint32_t newIRans=0;
uint32_t oldIRans=0;	
//uint32_t DCnewIRans=0;
//uint32_t DColdIRans=0;	
//int ALLans;
//int REDdata[100];
//int IRdata[100];
// int maxREDdata=0;
// int maxIRdata=0;
uint32_t SpO2ans;
uint8_t FIFOread;
uint8_t FIFOWrite;
uint8_t oldFIFOWrite=0;

//myi2c.c
void i2c_start(void);
void i2c_stop(void);
uint8_t i2c_write(uint8_t d);
uint8_t i2c_read(uint8_t ack);

//------------------ HCLK ------------------
void init_HCLK(void){
  SYS_UnlockReg();
    CLK_EnableXtalRC(CLK_PWRCTL_HXT_EN_Msk);
    CLK_WaitClockReady( CLK_CLKSTATUS_HXT_STB_Msk);
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT,CLK_HCLK_CLK_DIVIDER(1));
  SYS_LockReg();
}

//------------------ Systick ------------------
uint32_t tick=0;

void SysTick_Handler(void)
{tick++;};

void delay_ms(uint32_t ms)
{
    ms += tick;
    while( ms != tick){}
}


//------------------ Timer0 ------------------
void init_TIMER0(int freq)
{
  SYS_UnlockReg();
    //--- CLK
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HXT, 0);
		//CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HIRC, 0);
    //--- PIN
  SYS_LockReg();
    //--- OPEN
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, freq);
    TIMER_Start(TIMER0);
    //--- NVIC
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);
}
    //--- ISR
void TMR0_IRQHandler(void)
{
		//printf("%d\r\n",IRans);
    TIMER_ClearIntFlag( TIMER0 );
}

//------------------ UART0 ------------------
void init_UART0(void){
	//CLOCK
  SYS_UnlockReg();
  CLK_EnableModuleClock(UART0_MODULE);
  CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_UART_CLK_DIVIDER(1));
  //PIN
  SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);	
  SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_RX | SYS_PB_L_MFP_PB1_MFP_UART0_TX);	
  SYS_LockReg();
  //OPEN
  UART_Open(UART0, 115200);
  UART_ENABLE_INT(UART0, UART_IER_RDA_IE_Msk);
  //NVIC
  NVIC_EnableIRQ(UART0_IRQn);
}

void UART0_IRQHandler(void)
{
  //ISR
  UART0->THR = UART0->RBR;
}

//----------- UART1 --------------
void init_UART1(void){
  SYS_UnlockReg();
    //--- CLK
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_SetModuleClock( UART1_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_UART_CLK_DIVIDER(1) );
    //--- PIN
    SYS->PB_L_MFP &=~ SYS_PB_L_MFP_PB4_MFP_Msk;
    SYS->PB_L_MFP |=  SYS_PB_L_MFP_PB4_MFP_UART1_RX;
    SYS->PB_L_MFP &=~ SYS_PB_L_MFP_PB5_MFP_Msk;
    SYS->PB_L_MFP |=  SYS_PB_L_MFP_PB5_MFP_UART1_TX;
  SYS_LockReg();
    //--- OPEN
    UART_Open(UART1, 9600);
    //--- NVIC
    UART_ENABLE_INT(UART1, UART_IER_RDA_IE_Msk);
    NVIC_EnableIRQ(UART1_IRQn);
}
  //--- IRQ
void UART1_IRQHandler(void){
    UART1->THR = UART1->RBR;
}



//------------------ INIT ------------------
void init(void)
{
  init_HCLK();
	SysTick_Config(SystemCoreClock/1000);
}

//------------------ I2C ------------------
#define RD 1 
#define WR 0
#define ACK 0
#define NACK 1

uint8_t partID;
uint8_t revisionID;
uint8_t mode;
uint8_t SPO2;
uint8_t MultiLED1;
uint8_t MultiLED2;
uint8_t fifo;
uint8_t FIFO;
uint8_t LED_RED;
uint8_t LED_IR;
uint8_t myFIFO_Write_Pointer;
uint8_t myFIFO_Read_Pointer;
uint8_t myFIFO_Overflow_Counter;
uint32_t MAX30102data[9];
uint32_t ALLans;

//------------------ initMax30102 ------------------
void initMax30102(void){
//get part ID
i2c_start();
i2c_write(0xAE + WR);
i2c_write(0xFF);
i2c_start();
i2c_write(0xAE + RD);
partID=i2c_read(NACK);
i2c_stop();
	
//get revision ID
i2c_start();
i2c_write(0xAE + WR);
i2c_write(0xFE);
i2c_start();
i2c_write(0xAE+RD);
revisionID=i2c_read(NACK);
i2c_stop();

delay_ms(2);
	
//Mode configuration
i2c_start();
i2c_write(0xAE + WR);
i2c_write(0x09);
i2c_start();
i2c_write(0xAE + RD);
mode=i2c_read(NACK);
i2c_stop();
//set mode 0x40
i2c_start();
i2c_write(0xAE + WR);
i2c_write(0x90);
i2c_write(0x40);
i2c_stop();
//read mode
i2c_start();
i2c_write(0xAE + WR);
i2c_write(0x09);
i2c_start();
i2c_write(0xAE+RD);
mode=i2c_read(NACK);
i2c_stop();

//FIFO Configuration
i2c_start();
i2c_write(0xAE + WR);
i2c_write(0x08);
i2c_start();
i2c_write(0xAE + RD);
mode=i2c_read(NACK);
i2c_stop();
//set 0x4F
i2c_start();
i2c_write(0xAE + WR);
i2c_write(0x08);
i2c_write(0x4F);
i2c_stop();
//read FIFO
i2c_start();
i2c_write(0xAE + WR);
i2c_write(0x08);
i2c_start();
i2c_write(0xAE + RD);
mode=i2c_read(NACK);
i2c_stop();
//set 0x5F
i2c_start();
i2c_write(0xAE +WR);
i2c_write(0x08);
i2c_write(0x5F);
i2c_stop();

//Set Mode to MultiLED
i2c_start();
i2c_write(0xAE + WR);
i2c_write(0x09);
i2c_start();
i2c_write(0xAE + RD);
mode=i2c_read(NACK);
i2c_stop();
//set Mode to 0x07
i2c_start();
i2c_write(0xAE + WR);
i2c_write(0x09);
i2c_write(0x07);
i2c_stop();

//SPO2 Configuration
i2c_start();
i2c_write(0xAE + WR);
i2c_write(0x0A);
i2c_start();
i2c_write(0xAE+RD);
SPO2=i2c_read(NACK);
i2c_stop();
//set SPO2 to 0x20
i2c_start();
i2c_write(0xAE + WR);
i2c_write(0x0A);
i2c_write(0x20);
i2c_stop();
//read SPO2
i2c_start();
i2c_write(0xAE + WR);
i2c_write(0x0A);
i2c_start();
i2c_write(0xAE + RD);
SPO2=i2c_read(NACK);
i2c_stop();
//set SPO2 to 0x2C
i2c_start();
i2c_write(0xAE + WR);
i2c_write(0x0A);
i2c_write(0x2C);
i2c_stop();
//read SPO2
i2c_start();
i2c_write(0xAE + WR);
i2c_write(0x0A);
i2c_start();
i2c_write(0xAE + RD);
SPO2=i2c_read(NACK);
i2c_stop();
//set SPO2 to 0x2F
i2c_start();
i2c_write(0xAE + WR);
i2c_write(0x0A);
i2c_write(0x2F);			
//i2c_write(0x6C);		//######
i2c_stop();

//LED1,2,3 Pulse Amplitude Setting
i2c_start();
i2c_write(0xAE +WR);
i2c_write(0x0C);
i2c_write(0x1F);
i2c_stop();
i2c_start();
i2c_write(0xAE +WR);
i2c_write(0x0D);
i2c_write(0x1F);
i2c_stop();
i2c_start();
i2c_write(0xAE +WR);
i2c_write(0x0E);
i2c_write(0x1F);
i2c_stop();
i2c_start();
i2c_write(0xAE +WR);
i2c_write(0x10);
i2c_write(0x1F);
i2c_stop();

//MultiLED Mode Control 1
i2c_start();
i2c_write(0xAE +WR);
i2c_write(0x11);
i2c_start();
i2c_write(0xAE+RD);
MultiLED1=i2c_read(NACK);
i2c_stop();
//set MultiLED1 to 0x01
i2c_start();
i2c_write(0xAE +WR);
i2c_write(0x11);
i2c_write(0x01);
i2c_stop();
//read again
i2c_start();
i2c_write(0xAE +WR);
i2c_write(0x11);
i2c_start();
i2c_write(0xAE+RD);
MultiLED1=i2c_read(NACK);
i2c_stop();
//set MultiLED1 to 0x21
i2c_start();
i2c_write(0xAE +WR);
i2c_write(0x11);
i2c_write(0x21);
i2c_stop();

//MultiLED Mode Control 2
//MultiLED Mode Control 1
i2c_start();
i2c_write(0xAE +WR);
i2c_write(0x12);
i2c_start();
i2c_write(0xAE+RD);
MultiLED2=i2c_read(NACK);
i2c_stop();
//set MultiLED2 to 0x03
i2c_start();
i2c_write(0xAE +WR);
i2c_write(0x12);
i2c_write(0x03);
i2c_stop();

//RESET FIFO Writer Pointer/ Read Pointer / Overflow Counter
i2c_start();
i2c_write(0xAE +WR);
i2c_write(0x04);
i2c_write(0x00);
i2c_stop();
i2c_start();
i2c_write(0xAE +WR);
i2c_write(0x05);
i2c_write(0x00);
i2c_stop();
i2c_start();
i2c_write(0xAE +WR);
i2c_write(0x06);
i2c_write(0x00);
i2c_stop();

//Set LED Pulse Amplitude to 2mA
i2c_start();
i2c_write(0xAE +WR);
i2c_write(0x0C);
i2c_write(0x0A);
i2c_stop();
i2c_start();
i2c_write(0xAE +WR);
i2c_write(0x0E);
i2c_write(0x00);
i2c_stop();
}



int main(void)
{
  init();
	init_UART0();
	init_UART1();
	init_TIMER0(20);		// data output 20Hz							
	delay_ms(10);
	initMax30102();
	delay_ms(10);
	
  while(1)
	{
		delay_ms(1);
		i2c_start();
		i2c_write(0xAE +WR);
		i2c_write(0x06);
		i2c_start();
		i2c_write(0xAE+RD);
		FIFOread = i2c_read(NACK);
		i2c_stop();
		//--
		i2c_start();
		i2c_write(0xAE +WR);
		i2c_write(0x04);
		i2c_start();
		i2c_write(0xAE+RD);
		FIFOWrite = i2c_read(NACK);
		i2c_stop();

		/*//Reset FIFOread=0 if FOFOWrite=0
		if(FIFOWrite==0)
		{
			i2c_start();
			i2c_write(0xAE +WR);
			i2c_write(0x06);
			i2c_write(0x00);
			i2c_stop();
		}*/
		
		if(FIFOWrite != oldFIFOWrite)
		{
			//Read FIFO Data
			i2c_start();
			i2c_write(0xAE +WR);
			i2c_write(0x07);
			i2c_stop();
			i2c_start();
			//DATA
			i2c_write(0xAE +RD);
			MAX30102data[0] = i2c_read(ACK);//3Bytes RED (660nm)
			MAX30102data[1] = i2c_read(ACK);	
			MAX30102data[2] = i2c_read(ACK);	
			MAX30102data[3] = i2c_read(ACK);//3Bytes IR (880nm)
			MAX30102data[4] = i2c_read(ACK);	
			MAX30102data[5] = i2c_read(ACK);
			MAX30102data[6] = i2c_read(ACK);//3Bytes GREEN	
			MAX30102data[7] = i2c_read(ACK);	
			MAX30102data[8] = i2c_read(NACK);	
			i2c_stop();
			oldFIFOWrite = FIFOWrite;
			
			REDans = ( (MAX30102data[0] & 0x03) <<16) + (MAX30102data[1] << 8) + MAX30102data[2];			
			IRans = ( (MAX30102data[3] & 0x03) <<16) + (MAX30102data[4] << 8) + MAX30102data[5];	

			ALLans = ( REDans<<24 ) + IRans ;
								
			/*
			newREDans = ( (MAX30102data[0] & 0x03) <<16) + (MAX30102data[1] << 8) + MAX30102data[2];
			DCnewREDans = (newREDans + oldREDans)/2 ;			
			REDans = DCnewREDans - DColdREDans;
			oldREDans = newREDans;
			DColdREDans = DCnewREDans;
			
			newIRans = ( (MAX30102data[3] & 0x03) <<16) + (MAX30102data[4] << 8) + MAX30102data[5];			
			DCnewIRans = (newIRans + oldIRans)/2 ;			
			IRans = DCnewIRans - DColdIRans;
			oldIRans = newIRans;
			DColdIRans = DCnewIRans;			
			*/		
			
			/*
			newREDans = ( (MAX30102data[0] & 0x03) <<16) + (MAX30102data[1] << 8) + MAX30102data[2];
			REDans = (newREDans + oldREDans)/2 ;
			oldREDans = newREDans;																						
			newIRans = ( (MAX30102data[3] & 0x03) <<16) + (MAX30102data[4] << 8) + MAX30102data[5];			
			IRans = (newIRans + oldIRans)/2 ;		
			oldIRans = newIRans;
			*/			
			
			/*
			for(i=0;i<=98;i++)									//0:old 99:new
			{
				REDdata[i]=REDdata[i+1];
				IRdata[i]=IRdata[i+1];				
			}
			REDdata[99] = REDans;
			IRdata[99] = IRans;

			maxREDdata=REDdata[0];							// find max
			maxIRdata=IRdata[0];
			for(i=1;i<=99;i++)
			{
				if( REDdata[i] > maxREDdata )
				{
					maxREDdata = REDdata[i] ;
				}
				
				if( IRdata[i] > maxIRdata )
				{
					maxIRdata = IRdata[i] ;
				}					
			}
			*/
			if(REDans<5000) 
				{REDans = 0;}
			else 
				{REDans = REDans-5000;}
				
			if(IRans<12000) 
				{IRans = 0;}
			else 
				{IRans = IRans-12000;}
			
			//printf("%d\r\n",IRans);
			//printf("%d, ",IRans);	
			//printf("%d\r\n",ALLans); 
			printf("%d %d\r\n",REDans,IRans);


			//TMR0_IRQHandler();		
			//SpO2ans = 100 * REDans / (REDans + IRans);				// %
			//R = ( RED_AC / RED_DC ) / ( IR_AC / IR_DC )
			//SpO2 = -45.060*R*R + 30.354 *R + 94.845			
		}
  }
}



