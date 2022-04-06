/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "project_config.h"


/*_____ D E C L A R A T I O N S ____________________________________________*/


/*
	Target : 1K Freq
	DUTY : 50%
	
	SYS_CLK : 48M
	PSC : 100

	48 000 000/1 000 = PSC * (CNR + 1)
	CNR = (SYS_CLK/FREQ)/PSC - 1 = 479
	50 /1000 = CMR / (CNR + 1)
	CMR = 50 * (CNR + 1)/1000
	
*/

#define SYS_CLK 									(48000000ul)
#define PWM_PSC 								    (100)	
#define PWM_FREQ 								    (1000)	
#define PWM_DUTY                              	    (0)

#define PWM_CHANNEL                           	    (0)
#define PWM_CHANNEL_MASK                            (PWM_CH_0_MASK)

//16 bit
#define PWM_CNR 								    ((SYS_CLK/PWM_FREQ)/PWM_PSC - 1)
#define PWM_CMR 								    (PWM_DUTY * (PWM_CNR + 1)/1000)

#define CalNewDutyCMR(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)	    (u32DutyCycle * (PWM_GET_CNR(pwm, u32ChannelNum) + 1) / u32CycleResolution)
#define CalNewDuty(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)		(PWM_SET_CMR(pwm, u32ChannelNum, CalNewDutyCMR(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)))

/*_____ D E F I N I T I O N S ______________________________________________*/
volatile uint32_t BitFlag = 0;
volatile uint32_t counter_tick = 0;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void tick_counter(void)
{
	counter_tick++;
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void compare_buffer(uint8_t *src, uint8_t *des, int nBytes)
{
    uint16_t i = 0;	
	
    for (i = 0; i < nBytes; i++)
    {
        if (src[i] != des[i])
        {
            printf("error idx : %4d : 0x%2X , 0x%2X\r\n", i , src[i],des[i]);
			set_flag(flag_error , ENABLE);
        }
    }

	if (!is_flag_set(flag_error))
	{
    	printf("%s finish \r\n" , __FUNCTION__);	
		set_flag(flag_error , DISABLE);
	}

}

void reset_buffer(void *dest, unsigned int val, unsigned int size)
{
    uint8_t *pu8Dest;
//    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;

	#if 1
	while (size-- > 0)
		*pu8Dest++ = val;
	#else
	memset(pu8Dest, val, size * (sizeof(pu8Dest[0]) ));
	#endif
	
}

void copy_buffer(void *dest, void *src, unsigned int size)
{
    uint8_t *pu8Src, *pu8Dest;
    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;
    pu8Src  = (uint8_t *)src;


	#if 0
	  while (size--)
	    *pu8Dest++ = *pu8Src++;
	#else
    for (i = 0; i < size; i++)
        pu8Dest[i] = pu8Src[i];
	#endif
}

void dump_buffer(uint8_t *pucBuff, int nBytes)
{
    uint16_t i = 0;
    
    printf("dump_buffer : %2d\r\n" , nBytes);    
    for (i = 0 ; i < nBytes ; i++)
    {
        printf("0x%2X," , pucBuff[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");
}

void  dump_buffer_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for (i = 0; i < 16; i++)
            printf("%02X ", pucBuff[nIdx + i]);
        printf("  ");
        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
}

void PWM_Set_Duty(PWM_T *pwm,uint32_t u32ChannelNum,uint32_t u32DutyCycle,uint32_t u32CycleResolution)		// 1 ~ 1000 , 0.1 % to 100%
{
    uint32_t u32NewCMR = 0;
    u32NewCMR = CalNewDutyCMR(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution);    
    PWM_SET_CMR(pwm, u32ChannelNum, u32NewCMR);
}

void PWM1_Init(void)
{
    /*
    Configure PWM1 channel 0 init period and duty(down counter type).
    Period is PLL / (prescaler * (CNR + 1))
    Duty ratio = CMR / (CNR + 1)

    Period = 48 MHz / (1 * (199 + 1)) = 240000 Hz
    Duty ratio = 100 / (199 + 1) = 50%
    */

    /* Set PWM1 timer clock prescaler */
    PWM_SET_PRESCALER(PWM1, PWM_CHANNEL, PWM_PSC - 1);

    /* Set up counter type */
    PWM1->CTL1 &= ~PWM_CTL1_CNTTYPE0_Msk;

    /* Set PWM1 timer period */
    PWM_SET_CNR(PWM1, PWM_CHANNEL, PWM_CNR);

    /* Set PWM1 timer duty */
    PWM_SET_CMR(PWM1, PWM_CHANNEL, PWM_CMR);	

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    PWM_SET_OUTPUT_LEVEL(PWM1, PWM_CHANNEL_MASK, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);

    /* Enable output of PWM1 channel 0 */
    PWM_EnableOutput(PWM1, PWM_CHANNEL_MASK);

    PWM_Start(PWM1, PWM_CHANNEL_MASK);

    set_flag(flag_reverse , ENABLE);
}

void GPIO_Init (void)
{
   SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk)) | (SYS_GPB_MFPH_PB14MFP_GPIO);
		
   GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);

   GPIO_SetMode(PB, BIT15, GPIO_MODE_OUTPUT);	
}

void loop(void)
{
    static uint16_t duty = 500;	// 1 ~ 1000 , 0.1 % to 100%

    if (is_flag_set(flag_duty_change) )
    {
        set_flag(flag_duty_change ,DISABLE);

        // PWM_Set_Duty(PWM1, PWM_CHANNEL, duty, 1000);
        CalNewDuty(PWM1, PWM_CHANNEL, duty, 1000);

        if (is_flag_set(flag_reverse))
        {
            duty++;	
        }
        else
        {
            duty--;
        }

        if (duty == 1000)
        {
            set_flag(flag_reverse , DISABLE);				
        }
        else if (duty == 0)
        {
            set_flag(flag_reverse , ENABLE);
        }
    }

    if (is_flag_set(flag_duty_incr) )
    {
        set_flag(flag_duty_incr ,DISABLE); 

        duty = (duty >= 1000) ? (1000) : (duty + 1) ;
        CalNewDuty(PWM1, PWM_CHANNEL, duty, 1000);
    }    

     if (is_flag_set(flag_duty_decr) )
    {
        set_flag(flag_duty_decr ,DISABLE);  

        duty = (duty <= 1) ? (1) : (duty - 1) ;
        CalNewDuty(PWM1, PWM_CHANNEL, duty, 1000);
    }       

}

void TMR1_IRQHandler(void)
{
//	static uint32_t LOG = 0;

	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
//        	printf("%s : %4d\r\n",__FUNCTION__,LOG++);
			PB14 ^= 1;
		}

        if (is_flag_set(flag_duty_change_enable))
        {
            if ((get_tick() % 500) == 0)
            {
                set_flag(flag_duty_change ,ENABLE);
            }	
        }


		if ((get_tick() % 50) == 0)
		{

		}	
    }
}


void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res == 'x' || res == 'X')
	{
		NVIC_SystemReset();
	}

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
			case '1':
                set_flag(flag_duty_change_enable ,ENABLE);
                printf("flag_duty_change(1)\r\n");
				break;
                
			case '2':
                set_flag(flag_duty_change_enable ,DISABLE);     
                printf("flag_duty_change(0)\r\n");       
				break;

			case 'A':
			case 'a':            
                set_flag(flag_duty_incr ,ENABLE);
                printf("flag_duty_incr\r\n");
				break;
                
			case 'D':
			case 'd':            
                set_flag(flag_duty_decr ,ENABLE);    
                printf("flag_duty_decr\r\n");        
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
				NVIC_SystemReset();		
				break;
		}
	}
}

void UART0_IRQHandler(void)
{

    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
            UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(UART0_IRQn);
	
	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
	#endif	

}



void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);	

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);	

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    CLK_EnableModuleClock(TMR1_MODULE);
  	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    CLK_EnableModuleClock(PWM1_MODULE);
    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL2_PWM1SEL_PCLK1, 0);
    SYS_ResetModule(PWM1_RST);

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);


    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC5MFP_Msk)) | SYS_GPC_MFPL_PC5MFP_PWM1_CH0;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}


/*
 * This is a template project for M251 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */
int main()
{
    SYS_Init();

	UART0_Init();
	GPIO_Init();
	TIMER1_Init();
    PWM1_Init();

    /* Got no where to go, just loop forever */
    while(1)
    {
        loop();

    }
}


/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
