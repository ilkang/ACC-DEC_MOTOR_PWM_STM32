#include <stm32f10x.h>
#include <system_stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_usart.h>
#include <misc.h>
#include <stdio.h>

GPIO_InitTypeDef            GPIO_InitType;
EXTI_InitTypeDef            EXTI_InitType;
TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitType;
TIM_OCInitTypeDef           TIM_OCInitType;
TIM_ICInitTypeDef           TIM_ICInitStructure;
USART_InitTypeDef           USART_InitType;
NVIC_InitTypeDef            NVIC_InitType;

static __IO uint32_t TimingDelay;
 
/******변수 선언*****/ 
//Motor Status
uint8_t STATUS=0x11;                                // Motor Status [Define:0x11, Init:0x01, Move:0x02, Countinuous Move:0x03]
//uint16_t MOTOR_STATUS=0x0000;                     // Motor Moving Status
uint8_t MOTOR1_STATUS=0x00;                         // Motor #1 Max Status [Define:0x00, Move:0x11, Stop:0x22]
uint8_t MOTOR2_STATUS=0x00;                         // Motor #2 Max Status [Define:0x00, Move:0x11, Stop:0x22]
uint16_t PHOTO1_STATUS=SET;                         // Photo Sensor #1 Status
uint16_t PHOTO2_STATUS=SET;                         // Photo Sensor #2 Status

//uint16_t PWM=1080;
uint16_t T3_PWM=0;
uint16_t T3_PWM_Acc=1080;
uint16_t T3_PWM_Dec=21600;
uint16_t T3_PWM_Period=1080;

uint8_t MOTORT3_STATUS=0x00;

// Zoom & Focus Motor Position
//int16_t Cap_Focus_val=0, Cap_Clock_Fval=0, Cap_Unclock_Fval=0;
//int32_t Cap_Zoom_val=0, Cap_Clock_Zval=0, Cap_Unclock_Zval=0;

// Zoom & Focus Motor Init Flag
//uint8_t Focus_flg=0, Zoom_flg=0;

// Zoom & Focus Motor USART Function
uint8_t Com=0, Focus_dir=0, Zoom_dir=0;
uint8_t ref_Value_3=0, ref_Value_4=0;
int16_t ref_Value_1=0;
int32_t ref_Value_2=0;
uint8_t velc=0, velr=0;
uint16_t Enc_count=0;

// USART1 Function declaration
uint8_t COM_status=0;
uint8_t MSG[3]={0,0,0};
uint8_t DATA[5]={0,0,0,0,0};
uint8_t msg_count=0;
uint8_t data_count=0;

// USART3 Function declaration
uint8_t COM_status3=0;
uint8_t MSG3[8]={0,0,0,0,0,0,0,0};
uint8_t msg_count3=0;

uint16_t Focus_STEP=0, Focus_Ref_STEP=0, First_Focus_STEP=0, Second_Focus_STEP, Mid_Focus_Ref_STEP=0;
uint16_t Zoom_STEP=0, Zoom_Ref_STEP=0, First_Zoom_STEP=0, Second_Zoom_STEP=0;
uint8_t MSG_ID=0, MSG_LENGTH=0;
float Focus_Value=0.0;
int i=0;

/******함수 선언*****/
// MCU Initial
void init_mcu(void);
// ms #1 Delay Function
void delay_ms_sys(uint32_t val);
// USART1 1byte Transmit
void Putch1(uint16_t val);
// USART3 1byte Transmit
void Putch3(uint16_t val);
// USART3 1byte Receive
uint8_t Getch3(void);
void Wtire_Motor(uint16_t Focus_Ref);
void TIM3_Init(uint16_t Period);

// Main
void main(void)
{
    // MCU Initial
    init_mcu();
    delay_ms_sys(10);     
        
    while(1){
       
    }        
}

// Motor Driving Interrupt
void TIM1_UP_IRQHandler(void)
{
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

    /*Enc_count++;
    if(Enc_count==1250)
    {
        Putch3(0x3E);     
        Putch3((uint8_t)(MOTOR1_STATUS));  
        Putch3((uint8_t)(MOTOR2_STATUS));      
        Putch3((uint8_t)(((int16_t)Focus_STEP&0xFF00)>>8));                         
        Putch3((uint8_t)((int16_t)Focus_STEP&0x00FF)); 
        Putch3((uint8_t)(((int16_t)Zoom_STEP&0xFF00)>>8));                         
        Putch3((uint8_t)((int16_t)Zoom_STEP&0x00FF)); 
        Putch3((uint8_t)(((int16_t)Focus_Ref_STEP&0xFF00)>>8));                         
        Putch3((uint8_t)((int16_t)Focus_Ref_STEP&0x00FF));  
        Putch3((uint8_t)(((int16_t)Zoom_Ref_STEP&0xFF00)>>8));                         
        Putch3((uint8_t)((int16_t)Zoom_Ref_STEP&0x00FF));
        Putch3(0xE3);
    
        Enc_count=0;
    }*/

    Wtire_Motor(Focus_Ref_STEP);
}

// RS-232 Receive Interrupt
void USART1_IRQHandler(void)
{
    uint8_t val=0;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {  
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        val=(USART_ReceiveData(USART1) & 0x00FF);
    }
}

// RS-422 Receive Interrupt
void USART3_IRQHandler(void)
{
    uint8_t val=0;

    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {  
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);

        val=(USART_ReceiveData(USART3) & 0x00FF);

        if(COM_status==0){
            if(val==0x7E){COM_status=1;}
        }
        else if(COM_status==1){ 
            MSG[msg_count]=val;
                   
            if(msg_count==1){
                //MSG_ID=((((int16_t)MSG[0]<<8)+(int16_t)MSG[1]));
                MSG_ID=MSG[0];
                MSG_LENGTH=MSG[1];
                COM_status=2;
            }
        
            msg_count++;
        }
        else if(COM_status==2){ 
            DATA[data_count]=val;
            data_count++;
        
            if(data_count==MSG_LENGTH)
            {
                //R_checksum=DATA[MSG_LENGTH+1];
                //checksum=checksum(MSG_ID,MSG_LENGTH,DATA[0],DATA[1],DATA[2],DATA[3],DATA[4])
                switch(MSG_ID)
                {
                    case 0x01 : //if(R_checksum==checksum){
                                    //STATUS=0x02;
                                    //MOTOR1_STATUS=0x11;
                                    //MOTOR2_STATUS=0x11;
                                    MOTORT3_STATUS=0x01;
                                    T3_PWM=21600;
                                    //Focus_Ref_STEP=1000;
                                    Focus_Ref_STEP=((((int16_t)DATA[0]<<8)+(int16_t)DATA[1]));
                                    Zoom_Ref_STEP=((((int16_t)DATA[2]<<8)+(int16_t)DATA[3]));
                                    Mid_Focus_Ref_STEP=(Focus_STEP+Focus_Ref_STEP)/2;
                                    COM_status=0;
                                    msg_count=0;
                                    data_count=0;
                                //}
                                break;//
                    default : 
                              break;
                }
            }       
        }
    }
}

// MCU Initial
void init_mcu(void)
{
    // Clock Setting
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_USART1, ENABLE);
    
    // Systick Settinig
    SysTick->LOAD=SystemCoreClock/1000;
    SysTick->CTRL=SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    
    // GPIOA Setting(Zoom motor Step->A2, Zoom motor Direction->A3)
    GPIO_InitType.GPIO_Mode=GPIO_Mode_AF_PP;
    GPIO_InitType.GPIO_Pin=GPIO_Pin_2;
    GPIO_InitType.GPIO_Speed=GPIO_Speed_50MHz;

    GPIO_Init(GPIOA, &GPIO_InitType);

    GPIO_InitType.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_InitType.GPIO_Pin=GPIO_Pin_3;
    GPIO_InitType.GPIO_Speed=GPIO_Speed_50MHz;

    GPIO_Init(GPIOA, &GPIO_InitType);
    GPIO_Write(GPIOA, 0x0000);

    // GPIOA Setting(USART1) 
    GPIO_InitType.GPIO_Mode=GPIO_Mode_AF_PP;
    GPIO_InitType.GPIO_Pin=GPIO_Pin_9;
    GPIO_InitType.GPIO_Speed=GPIO_Speed_50MHz;

    GPIO_Init(GPIOA, &GPIO_InitType);
    
    GPIO_InitType.GPIO_Mode=GPIO_Mode_IN_FLOATING;
    GPIO_InitType.GPIO_Pin=GPIO_Pin_10;

    GPIO_Init(GPIOA, &GPIO_InitType);

    // GPIOB Setting(USART3) 
    GPIO_InitType.GPIO_Mode=GPIO_Mode_AF_PP;
    GPIO_InitType.GPIO_Pin=GPIO_Pin_10;
    GPIO_InitType.GPIO_Speed=GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitType);
    
    GPIO_InitType.GPIO_Mode=GPIO_Mode_IN_FLOATING;
    GPIO_InitType.GPIO_Pin=GPIO_Pin_11;

    GPIO_Init(GPIOB, &GPIO_InitType);
    
    // GPIOB Setting(Focus motor Direction->B9)
    GPIO_InitType.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_InitType.GPIO_Pin=GPIO_Pin_9;
    GPIO_InitType.GPIO_Speed=GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitType);
    GPIO_Write(GPIOB, 0x0000);

    // GPIOC Setting(Debug)
    GPIO_InitType.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_InitType.GPIO_Pin=GPIO_Pin_0;
    GPIO_InitType.GPIO_Speed=GPIO_Speed_2MHz;

    GPIO_Init(GPIOC, &GPIO_InitType);
    GPIO_Write(GPIOC, 0x0000);
    
    // GPIOC Setting(Photo Sensor Zoom->C1, Focus->C2)
    GPIO_InitType.GPIO_Pin=GPIO_Pin_1, GPIO_Pin_2;
    GPIO_InitType.GPIO_Mode=GPIO_Mode_IN_FLOATING;
    GPIO_InitType.GPIO_Speed=GPIO_Speed_50MHz;
    
    GPIO_Init(GPIOC, &GPIO_InitType);
    
    // GPIOC Setting(Focus motor Step->C9)
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
    GPIO_InitType.GPIO_Mode=GPIO_Mode_AF_PP;
    GPIO_InitType.GPIO_Pin=GPIO_Pin_9;
    GPIO_InitType.GPIO_Speed=GPIO_Speed_50MHz;

    GPIO_Init(GPIOC, &GPIO_InitType);
    GPIO_Write(GPIOC, 0x0000);
    
    // GPIOC Setting(Zoom motor Enable->C3)
    GPIO_InitType.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_InitType.GPIO_Pin=GPIO_Pin_3;
    GPIO_InitType.GPIO_Speed=GPIO_Speed_50MHz;

    GPIO_Init(GPIOC, &GPIO_InitType);
    GPIO_Write(GPIOC, 0x0000);
    
    // GPIOD Setting(Focus motor Enable->D2)
    GPIO_InitType.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_InitType.GPIO_Pin=GPIO_Pin_2;
    GPIO_InitType.GPIO_Speed=GPIO_Speed_50MHz;

    GPIO_Init(GPIOD, &GPIO_InitType);
    GPIO_Write(GPIOD, 0x0000);
    
    // Timer1 Setting(Motor Driving Interrupt)
    TIM_TimeBaseInitType.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseInitType.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseInitType.TIM_Period=0x086F;         // 360,000(TIME_CLK)/(TIM_Period+1)=1666.6           //0x0013(0.6ms)   
    TIM_TimeBaseInitType.TIM_Prescaler=0x0013;      // 72,000,000(system_clk)/(TIM_Prescaler+1)=TIME_CLK

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitType);
    
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    
    // Timer2 Setting(Zoom Motor Setting)
    TIM_TimeBaseInitType.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseInitType.TIM_CounterMode=TIM_CounterMode_Up;
    
    TIM_TimeBaseInitType.TIM_Period=0x0437;     // 72,000,000(TIME_CLK)/(TIM_Period+1)=66666.6         //0x0000, 0x0437(15us)
    TIM_TimeBaseInitType.TIM_Prescaler=0x0000;  // 72,000,000(system_clk)/(TIM_Prescaler+1)=TIME_CLK

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitType);

    TIM_OCInitType.TIM_OCMode=TIM_OCMode_PWM1;
    TIM_OCInitType.TIM_OutputState=TIM_OutputState_Enable;
    TIM_OCInitType.TIM_Pulse=0x0000;

    TIM_OC3Init(TIM2, &TIM_OCInitType);
    
    TIM_Cmd(TIM2, ENABLE);
    
    // Timer3 Setting(Focus Motor Setting)
    /*TIM_TimeBaseInitType.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseInitType.TIM_CounterMode=TIM_CounterMode_Up; 
    //TIM_TimeBaseInitType.TIM_Period=0x464F;     // 72,000,000(TIME_CLK)/(TIM_Period+1)=4000         //0x0000, 0x464F(250us)      
    //TIM_TimeBaseInitType.TIM_Period=0x1C1F;     // 72,000,000(TIME_CLK)/(TIM_Period+1)=10000         //0x0000, 0x1C1F(100us)
    //TIM_TimeBaseInitType.TIM_Period=0x02CF;     // 72,000,000(TIME_CLK)/(TIM_Period+1)=100000          //0x0000, 0x02CF(10us)
    //TIM_TimeBaseInitType.TIM_Period=0x0437;     // 72,000,000(TIME_CLK)/(TIM_Period+1)=66666.6         //0x0000, 0x0437(15us)
    TIM_TimeBaseInitType.TIM_Period=PWM-1;    
    TIM_TimeBaseInitType.TIM_Prescaler=0x0000;  // 72,000,000(system_clk)/(TIM_Prescaler+1)=TIME_CLK

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitType);
    
    TIM_OCInitType.TIM_OCMode=TIM_OCMode_PWM1;
    TIM_OCInitType.TIM_OutputState=TIM_OutputState_Enable;
    TIM_OCInitType.TIM_Pulse=0x0000;
    
    TIM_OC4Init(TIM3, &TIM_OCInitType);

    TIM_Cmd(TIM3, ENABLE);*/
    
    // USART Setting
    USART_InitType.USART_BaudRate=115200;
    USART_InitType.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
    USART_InitType.USART_Mode=USART_Mode_Tx | USART_Mode_Rx;
    USART_InitType.USART_Parity=USART_Parity_No;
    USART_InitType.USART_StopBits=USART_StopBits_1;
    USART_InitType.USART_WordLength=USART_WordLength_8b;

    USART_Init(USART1, &USART_InitType);
    
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
    
    USART_Init(USART3, &USART_InitType);
    
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART3, ENABLE);

    // Timer1 Interrupt Setting(Motor Driving Interrupt)
    NVIC_InitType.NVIC_IRQChannel=TIM1_UP_IRQn;
    NVIC_InitType.NVIC_IRQChannelCmd=ENABLE;
    NVIC_InitType.NVIC_IRQChannelPreemptionPriority=0x00;
    NVIC_InitType.NVIC_IRQChannelSubPriority=0x00;
    
    NVIC_Init(&NVIC_InitType);
    
    // USART1 Interrupt Setting(PC Communication)
    NVIC_InitType.NVIC_IRQChannel=USART1_IRQn;
    NVIC_InitType.NVIC_IRQChannelCmd=ENABLE;
    NVIC_InitType.NVIC_IRQChannelPreemptionPriority=3;
    NVIC_InitType.NVIC_IRQChannelSubPriority=4;
    
    NVIC_Init(&NVIC_InitType);
    
    // USART3 Interrupt Setting(PC Communication)
    NVIC_InitType.NVIC_IRQChannel=USART3_IRQn;
    NVIC_InitType.NVIC_IRQChannelCmd=ENABLE;
    NVIC_InitType.NVIC_IRQChannelPreemptionPriority=3;
    NVIC_InitType.NVIC_IRQChannelSubPriority=4;
    
    NVIC_Init(&NVIC_InitType);
    
}
// ms #1 Delay Function
void delay_ms_sys(uint32_t val)
{
    while(val){
        while(SysTick->VAL>1000);
        while(SysTick->VAL<1000);
        val--;
    }
}
// USART1 1byte Transmit
void Putch1(uint16_t val)
{
    while(!USART_GetFlagStatus(USART1, USART_FLAG_TXE));

    USART_SendData(USART1, val);
}
// USART3 1byte Transmit
void Putch3(uint16_t val)
{
    while(!USART_GetFlagStatus(USART3, USART_FLAG_TXE));

    USART_SendData(USART3, val);
}
// USART3 1byte Receive
uint8_t Getch3(void)
{
    uint16_t val=0;

    while(!USART_GetFlagStatus(USART3, USART_FLAG_RXNE));

    val=(USART_ReceiveData(USART3)&0x00FF);

    return val;
}

void Wtire_Motor(uint16_t Focus_Ref)
{
    switch(MOTORT3_STATUS)
    {
        case 0x01 : GPIO_Write(GPIOD, 0x0000);          // Focus motor Enable, PD2(54)

                    if(Focus_STEP<Mid_Focus_Ref_STEP){
                        GPIO_Write(GPIOB, 0x0200);          // Clockwise, PB9(62)
                        if(T3_PWM>T3_PWM_Acc) T3_PWM=T3_PWM-T3_PWM_Period;
                        else T3_PWM=T3_PWM_Acc;
                        TIM3_Init(T3_PWM);
                        TIM3->CCR4=(T3_PWM/2);
                        Focus_STEP++;
                    }
                    else if(Focus_STEP<Focus_Ref && Focus_STEP>Mid_Focus_Ref_STEP){
                        GPIO_Write(GPIOB, 0x0200);          // Clockwise, PB9(62)
                        if(T3_PWM<T3_PWM_Dec) T3_PWM=T3_PWM+T3_PWM_Period;
                        else T3_PWM=T3_PWM_Dec;
                        TIM3_Init(T3_PWM);
                        TIM3->CCR4=(T3_PWM/2);
                        Focus_STEP++;
                    }
                    else if(Focus_STEP>Mid_Focus_Ref_STEP){
                        GPIO_Write(GPIOB, 0x0000);          // UnClockwise, PB9(62)
                        if(T3_PWM>T3_PWM_Acc) T3_PWM=T3_PWM-T3_PWM_Period;
                        else T3_PWM=T3_PWM_Acc;
                        TIM3_Init(T3_PWM);
                        TIM3->CCR4=(T3_PWM/2);
                        Focus_STEP--;
                    }
                    else if(Focus_STEP<Mid_Focus_Ref_STEP && Focus_STEP>Focus_Ref ){
                        GPIO_Write(GPIOB, 0x0000);          // UnClockwise, PB9(62)
                        if(T3_PWM<T3_PWM_Dec) T3_PWM=T3_PWM+T3_PWM_Period;
                        else T3_PWM=T3_PWM_Dec;
                        TIM3_Init(T3_PWM);
                        TIM3->CCR4=(T3_PWM/2);
                        Focus_STEP--;
                    }
                    else{
                        TIM3->CCR4=0;
                        MOTORT3_STATUS=0x02;
                    }
                    break;
        case 0x02 : GPIO_Write(GPIOD, 0x0004);          // Focus motor Disable, PD2(54)
                    break;
        //case 0x03 : ;
        //            break;
        default : break;
    }
}

void TIM3_Init(uint16_t Period)
{
    TIM_TimeBaseInitType.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseInitType.TIM_CounterMode=TIM_CounterMode_Up; 
    TIM_TimeBaseInitType.TIM_Period=Period-1;    
    TIM_TimeBaseInitType.TIM_Prescaler=0x0000;  // 72,000,000(system_clk)/(TIM_Prescaler+1)=TIME_CLK

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitType);
    
    TIM_OCInitType.TIM_OCMode=TIM_OCMode_PWM1;
    TIM_OCInitType.TIM_OutputState=TIM_OutputState_Enable;
    TIM_OCInitType.TIM_Pulse=0x0000;
    
    TIM_OC4Init(TIM3, &TIM_OCInitType);

    TIM_Cmd(TIM3, ENABLE);
}