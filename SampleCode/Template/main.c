/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M031 MCU.
 *
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

/*----------------------------------------------------*/
typedef enum{
	flag_DEFAULT = 0 ,

	flag_0_23Hz , 
	flag_1Hz , 	
	flag_151Hz , 

	flag_reverse ,

	flag_END	
}Flag_Index;

volatile uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))

/*----------------------------------------------------*/
#define FIFO_THRESHOLD 							(4)
#define RX_BUFFER_SIZE 							(256)
#define RX_TIMEOUT_CNT 							(20)

typedef struct {
	uint8_t RX_Buffer[RX_BUFFER_SIZE];
	uint16_t Length;
	uint8_t RDA_Trigger_Cnt;
	uint8_t RXTO_Trigger_Cnt;

}UART_BUF_t;

UART_BUF_t uart0Dev;

/*----------------------------------------------------*/
/*
	Target : 200K Freq
	DUTY : 50%
	
	SYS_CLK : 48M
	PSC : 1

	48 000 000/200 000 = PSC * (CNR + 1)
	CNR = (SYS_CLK/FREQ)/PSC - 1 = 239

	50 /100 = CMR / (CNR + 1)
	CMR = 50 * (CNR + 1)/100
	
*/

#define SYS_CLK 									(48000000ul)
#define PWM_PSC 								(10)	
#define PWM_FREQ 								(100ul)	
#define PWM_DUTY 								(50)

#define PWM_CH									(0)
#define PWM_CH_MASK							(PWM_CH_0_MASK)

//16 bit
#define PWM_CNR 								((SYS_CLK/PWM_FREQ)/PWM_PSC - 1)
#define PWM_CMR 								(PWM_DUTY * (PWM_CNR + 1)/1000)

#define CalNewDutyCMR(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)	(u32DutyCycle * (PWM_GET_CNR(pwm, u32ChannelNum) + 1) / u32CycleResolution)
#define CalNewDuty(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)		(PWM_SET_CMR(pwm, u32ChannelNum, CalNewDutyCMR(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)))

uint32_t g_au32Freq = PWM_FREQ;

/*----------------------------------------------------*/
#define dPDMA_TEST_LENGTH 						(2)
#define PDMAchannel_TIMER_EXT 					(1)
volatile uint32_t g_au32CAPValue[dPDMA_TEST_LENGTH];

#define PDMATIMEOUT(ms)						(ms*(CLK_GetCPUFreq()>>15)/1000)

#define TIMER_PSC	 							(0xFF)		// 0 ~ 0xFF

//#define USE_INT
#define USE_FLOAT

/*----------------------------------------------------*/
#define LED_TOGGLE								(PB14 = ~PB14)
#define TIMER_1S									(1000)	//(4302)	//(6)
uint16_t counter_target = TIMER_1S;


/*----------------------------------------------------*/

void PDMA_TimerCapture_Start(void)
{
    /* Transfer count is PDMA_TEST_LENGTH, transfer width is 8 bits(one byte) */
    PDMA_SetTransferCnt(PDMA, PDMAchannel_TIMER_EXT, PDMA_WIDTH_32, dPDMA_TEST_LENGTH);
    /* Set source address is au8SrcArray, Source increment size is 8 bits(one byte), destination address is PA->DOUT (no increment)*/
    PDMA_SetTransferAddr(PDMA, PDMAchannel_TIMER_EXT, (uint32_t) &(TIMER1->CAP), PDMA_SAR_FIX, (uint32_t)g_au32CAPValue, PDMA_DAR_INC);
    /* Request source is timer 1 */
    PDMA_SetTransferMode(PDMA, PDMAchannel_TIMER_EXT, PDMA_TMR1, FALSE,(uint32_t) NULL);
    /* Transfer type is burst transfer and burst size is 4 */
    PDMA_SetBurstType(PDMA, PDMAchannel_TIMER_EXT, PDMA_REQ_SINGLE,(uint32_t) NULL);
}


/*
	due to TIMER_SET_CMP_VALUE 2nd parameter limit : 0xFFF FFF
	=> able to detect lowest freq = 48000000 / 0xFFF FFF = 3
	=> able to detect lowest freq = 48000000(PSC + 1) / 0xFFF FFF = 0.00... , PSC = 0 ~ 0xFF
*/

void PDMA_TimerCapture_Init(void)	//PA10 : TM1_EXT 
{
    /* Enable Timer1 external capture function */
    TIMER_Open(TIMER1, TIMER_CONTINUOUS_MODE, 1);
    TIMER_SET_PRESCALE_VALUE(TIMER1, TIMER_PSC);
    TIMER_SET_CMP_VALUE(TIMER1, 0xFFFFFF);
    TIMER_EnableCapture(TIMER1, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_FALLING_EDGE);
    TIMER_SetTriggerSource(TIMER1, TIMER_TRGSRC_CAPTURE_EVENT);
    TIMER_SetTriggerTarget(TIMER1, TIMER_TRG_TO_PDMA);

    /* Open Channel 1 */
    PDMA_Open(PDMA, 1 << PDMAchannel_TIMER_EXT);
    /* Transfer count is PDMA_TEST_LENGTH, transfer width is 8 bits(one byte) */
    PDMA_SetTransferCnt(PDMA, PDMAchannel_TIMER_EXT, PDMA_WIDTH_32, dPDMA_TEST_LENGTH);
    /* Set source address is au8SrcArray, Source increment size is 8 bits(one byte), destination address is PA->DOUT (no increment)*/
    PDMA_SetTransferAddr(PDMA, PDMAchannel_TIMER_EXT, (uint32_t) &(TIMER1->CAP), PDMA_SAR_FIX, (uint32_t)g_au32CAPValue, PDMA_DAR_INC);
    /* Request source is timer 1 */
    PDMA_SetTransferMode(PDMA, PDMAchannel_TIMER_EXT, PDMA_TMR1, FALSE,(uint32_t) NULL);
    /* Transfer type is burst transfer and burst size is 4 */
    PDMA_SetBurstType(PDMA, PDMAchannel_TIMER_EXT, PDMA_REQ_SINGLE,(uint32_t) NULL);

	PDMA_SetTimeOut(PDMA,PDMAchannel_TIMER_EXT, TRUE, PDMATIMEOUT(1));
	PDMA_EnableTimeout(PDMA,(1 << PDMAchannel_TIMER_EXT));

    /* Enable interrupt */
    PDMA_EnableInt(PDMA, PDMAchannel_TIMER_EXT, PDMA_INT_TRANS_DONE);
    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA_IRQn);

    TIMER_Start(TIMER1);
	
}


void PDMA_TimerCapture_Process(void)
{
    uint32_t u32InitCount = 0;
    volatile uint32_t u32CAPDiff = 0;
    volatile uint32_t res = 0;	
	
	#if defined (USE_INT)
    uint32_t Freq = 0;
	#elif defined (USE_FLOAT)
    double Freq = 0;
	#endif
	
    for(u32InitCount = 1; u32InitCount < dPDMA_TEST_LENGTH; u32InitCount++)
    {
        u32CAPDiff = g_au32CAPValue[u32InitCount] - g_au32CAPValue[u32InitCount - 1];
		res = (SystemCoreClock / (TIMER_PSC + 1));
			
		if (g_au32CAPValue[u32InitCount] > g_au32CAPValue[u32InitCount - 1])
		{
			#if defined (USE_INT)
			Freq = (res/u32CAPDiff);			
			
			printf("%8d , %8d , clock : %6d , Diff: %9d. Freq: %6d Hz\r\n",g_au32CAPValue[u32InitCount], g_au32CAPValue[u32InitCount - 1],res,u32CAPDiff, Freq);
			#elif defined (USE_FLOAT) 
			Freq = ((double )res/(double )u32CAPDiff);
			
			printf("%8d , %8d , clock : %6d , Diff: %9d. Freq: %.2f Hz\r\n",g_au32CAPValue[u32InitCount], g_au32CAPValue[u32InitCount - 1],res,u32CAPDiff, Freq);
			#endif
		}
    }

	PDMA_TimerCapture_Start();//PDMA_TimerCapture_Init();
}

void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if(status & PDMA_INTSTS_REQTOF1_Msk)	//(1 << (PDMA_INTSTS_REQTOF1_Pos + UART_RX_PDMA_CH) /* Request Time-out */
    {
		PDMA_SetTimeOut(PDMA,1, 0, 0);
	
        /* Clear PDMA timeout interrupt flag */
        PDMA_CLR_TMOUT_FLAG(PDMA, 0);

		PDMA_SetTimeOut(PDMA,0, TRUE, PDMATIMEOUT(1));
        /* Disable and enable timeout function to restart the timeout counter */
		PDMA_DisableTimeout(PDMA,(1 << 0) );
		PDMA_EnableTimeout(PDMA,(1 << 0) );
		
        /* Set transfer count and trigger again */
        PDMA_SetTransferCnt(PDMA, 0, PDMA_WIDTH_32, dPDMA_TEST_LENGTH);
        /* Get the latest status for SPI PDMA again */
        status = PDMA_GET_INT_STATUS(PDMA);
    }


    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        /* Check if channel 1 has abort error */
        if (PDMA_GET_ABORT_STS(PDMA) & PDMA_ABTSTS_ABTIF1_Msk)
        {
			//abort
			printf("abort\r\n");
        }

        /* Clear abort flag of channel 1 */
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_ABTSTS_ABTIF1_Msk);
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        /* Check transmission of channel 1 has been transfer done */
        if (PDMA_GET_TD_STS(PDMA) & PDMA_TDSTS_TDIF1_Msk)
        {
			//done
			TIMER_ClearCaptureIntFlag(TIMER1);
//			TIMER_Stop(TIMER1);
			
			PDMA_TimerCapture_Process();

        }

        /* Clear transfer done flag of channel 1 */
        PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF1_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

void TMR1_IRQHandler(void)
{
    if(TIMER_GetCaptureIntFlag(TIMER1) == 1)
    {
        /* Clear Timer1 capture trigger interrupt flag */
        TIMER_ClearCaptureIntFlag(TIMER1);
    }
}

//void PWM_Set_Duty(PWM_T *pwm,uint32_t u32ChannelNum,uint32_t u32DutyCycle,uint32_t u32CycleResolution)		// 1 ~ 1000 , 0.1 % to 100%
//{
//    uint32_t u32NewCMR = 0;
//	u32NewCMR = CalNewDutyCMR(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution);    
//	PWM_SET_CMR(pwm, u32ChannelNum, u32NewCMR);
//}


void PWM0_Init(void)
{
    /*
      Configure PWM0 channel 0 init period and duty(down counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = CMR / (CNR + 1)
      Period = 48 MHz / (1 * (199 + 1)) = 240000 Hz
      Duty ratio = 100 / (199 + 1) = 50%
    */

	#if 0
    /* Set PWM0 timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, PWM_CH, PWM_PSC - 1);

    /* Set up counter type */
    PWM0->CTL1 &= ~PWM_CTL1_CNTTYPE0_Msk;

    /* Set PWM0 timer period */
    PWM_SET_CNR(PWM0, PWM_CH, PWM_CNR);

    /* Set PWM0 timer duty */
    PWM_SET_CMR(PWM0, PWM_CH, PWM_CMR);	

	#else
	PWM_ConfigOutputChannel(PWM0, PWM_CH, PWM_FREQ, PWM_DUTY);
	#endif

    /* Enable output of PWM0 channel 0 */
    PWM_EnableOutput(PWM0, PWM_CH_MASK);

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    PWM_SET_OUTPUT_LEVEL(PWM0, PWM_CH_MASK, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW);

	PWM_Start(PWM0, PWM_CH_MASK);

	set_flag(flag_reverse , ENABLE);

}


void GPIO_Init (void)
{
    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);
}


/*

	0.23Hz = 4.3 sec
	1Hz = 1 sec
	151Hz = 0.006

*/


void TMR3_IRQHandler(void)
{
	static uint16_t CNT = 0;
//	static uint16_t counter_test = 0;

	
    if(TIMER_GetIntFlag(TIMER3) == 1)
    {
        TIMER_ClearIntFlag(TIMER3);

		if (is_flag_set(flag_0_23Hz))
		{
			counter_target = 4302; 
		}
		else if (is_flag_set(flag_1Hz))
		{
			counter_target = 1000; 
		}
		else if (is_flag_set(flag_151Hz))
		{
			counter_target = 6; 
		}
	
		if (CNT++ >= counter_target)
		{		
			CNT = 0;
			LED_TOGGLE; //PB14 ^= 1;
		}

		#if 0
		if (counter_test++ >= 10)
		{		
			counter_test = 0;


			if (is_flag_set(flag_reverse))
			{
				g_au32Freq++;	
			}
			else
			{
				g_au32Freq--;
			}

			if (g_au32Freq == 1100)
			{
				set_flag(flag_reverse , DISABLE);				
			}
			else if (g_au32Freq == 0)
			{
				set_flag(flag_reverse , ENABLE);
			}

			PWM_ConfigOutputChannel(PWM0, PWM_CH, g_au32Freq, PWM_DUTY);
			PDMA_TimerCapture_Start();

		}	
		#endif
				
    }
}

void TIMER3_Init(void)
{
    TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER3);
    NVIC_EnableIRQ(TMR3_IRQn);	
    TIMER_Start(TIMER3);
}

void UART0_Process(uint8_t rx)
{
	switch(rx)
	{
		case 'a':
		case 'A':
			g_au32Freq = (g_au32Freq >= 24000000ul) ? (24000000ul) : (g_au32Freq + 100) ;	
			printf("Freq : %4d \r\n" , g_au32Freq);
			PWM_ConfigOutputChannel(PWM0, PWM_CH, g_au32Freq, PWM_DUTY);
			PDMA_TimerCapture_Start();
		break;

		case 'd':
		case 'D':
			g_au32Freq = (g_au32Freq <= 0) ? (0) : (g_au32Freq - 1) ;	
			printf("Freq : %4d \r\n" , g_au32Freq);
			PWM_ConfigOutputChannel(PWM0, PWM_CH, g_au32Freq, PWM_DUTY);
			PDMA_TimerCapture_Start();
		break;	

		case '1':				
			set_flag(flag_0_23Hz , ENABLE);
		break;

		case '2':
			set_flag(flag_1Hz , ENABLE);
		break;

		case '3':
			set_flag(flag_151Hz , ENABLE);
		break;			


		case 'Z':
		case 'z':
			NVIC_SystemReset();
		break;
		
	}
}

void UART02_IRQHandler(void)
{
    uint8_t i;
    static uint16_t u16UART_RX_Buffer_Index = 0;

	if ((UART_GET_INT_FLAG(UART0,UART_INTSTS_RDAINT_Msk)))
	{
        /* UART receive data available flag */
        
        /* Record RDA interrupt trigger times */
        uart0Dev.RDA_Trigger_Cnt++;
        
        /* Move the data from Rx FIFO to sw buffer (RAM). */
        /* Every time leave 1 byte data in FIFO for Rx timeout */
        for(i = 0 ; i < (FIFO_THRESHOLD - 1) ; i++)
        {
            uart0Dev.RX_Buffer[u16UART_RX_Buffer_Index] = UART_READ(UART0);
            u16UART_RX_Buffer_Index ++;

            if (u16UART_RX_Buffer_Index >= RX_BUFFER_SIZE) 
                u16UART_RX_Buffer_Index = 0;
        }	
	}
    else if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RXTOINT_Msk)) 
    {
        /* When Rx timeout flag is set to 1, it means there is no data needs to be transmitted. */

        /* Record Timeout times */
        uart0Dev.RXTO_Trigger_Cnt++;

        /* Move the last data from Rx FIFO to sw buffer. */
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
            uart0Dev.RX_Buffer[u16UART_RX_Buffer_Index] = UART_READ(UART0);
            u16UART_RX_Buffer_Index ++;
        }

        /* Clear UART RX parameter */
        UART_DISABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
		uart0Dev.Length = u16UART_RX_Buffer_Index;
        u16UART_RX_Buffer_Index = 0;

		#if 1
		printf("UART RX : 0x%2X \r\n" , uart0Dev.RX_Buffer[0]);
		
		UART0_Process(uart0Dev.RX_Buffer[0]);
		#else

		printf("\r\nUART0 Rx Received Data : %s\r\n",uart0Dev.RX_Buffer);
		printf("UART0 Rx Received Len : %d\r\n",uart0Dev.Length);	
		printf("UART0 Rx RDA (Fifofull) interrupt times : %d\r\n",uart0Dev.RDA_Trigger_Cnt);
		printf("UART0 Rx RXTO (Timeout) interrupt times : %d\r\n",uart0Dev.RXTO_Trigger_Cnt);
		#endif

		/* Reset UART interrupt parameter */
		UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
		memset(&uart0Dev, 0x00, sizeof(UART_BUF_t));
		
    }
	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, RX_TIMEOUT_CNT);

	/* Set UART FIFO RX interrupt trigger level to 4-bytes*/
    UART0->FIFO = ((UART0->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_4BYTES);

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART02_IRQn);	

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());

	UART_WAIT_TX_EMPTY(UART0);
	memset(&uart0Dev, 0x00, sizeof(UART_BUF_t));
	
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
	
    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
	
    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));
//    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));
	
    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));

    /* Enable PDMA clock */
    CLK_EnableModuleClock(PDMA_MODULE);

    CLK_EnableModuleClock(TMR1_MODULE);
   	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0, 0);
	
    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_PCLK1, 0);
	
    CLK_EnableModuleClock(PWM0_MODULE);
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, 0);
    /* Reset PWM0 module */
    SYS_ResetModule(PWM0_RST);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);


	/*----------------------------------------------------*/
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) |
                    SYS_GPB_MFPL_PB5MFP_PWM0_CH0;


	/*----------------------------------------------------*/
    /* Set multi-function pin for Timer1 external capture pin */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA10MFP_Msk)) \
                    | SYS_GPA_MFPH_PA10MFP_TM1_EXT;	

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M031 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

    UART0_Init();

	GPIO_Init();		//PB14 : LED
	
	TIMER3_Init();	

    PWM0_Init();	//PB5 : PWM0_CH0

	PDMA_TimerCapture_Init();	//PA10 : TM1_EXT 

    /* Got no where to go, just loop forever */
    while(1)
    {


    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
