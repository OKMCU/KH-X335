// Project: KH-X335.prj
// Device:  MS83F0802
// Memory:  Flash 2KX14b, EEPROM 256X8b, SRAM 128X8b
// Author:  SUN Wentao
// Company: Pencil Development
// Version: V1.0
// Date:    18 May 2019
//===========================================================
//                      PIN mapping
//                 ---------------------
//  VDD ----------|1(VDD)         (GND)8|------------ GND
//  MOTOR_A ------|2(PA6)     (PA0/CLK)7|------------ MOTOR_D
//  MOTOR_B ------|3(PC3)     (PA1/DAT)6|------------ MOTOR_C
//  AD_VR --------|4(PC2/AN6) (PC1/AN5)5|------------ AD_KEY
//                 ---------------------
//===========================================================
#include	"SYSCFG.h"
#include 	"MS83Fxx02.h"
#include    "stdint.h"

//===========================================================
//                          CONSTANTS
//===========================================================
#define _XTAL_FREQ 		4000000
#define TIMER0_VALUE    6
#define NULL            0
#define SPEED_MAX       9
#define SPEED_MIN       59
#define DIR_FRWD        0
#define DIR_BKWD        1
#define ADC_CHANNEL_VR  6
#define ADC_CHANNEL_KEY 5
#define KEY_PWR         0x01
#define KEY_DIR         0x02
#define KEY_DEBOUNCE    200


#define GPIO_MOTOR_A    RA6     //蓝
#define GPIO_MOTOR_B    RC3     //粉
#define GPIO_MOTOR_C    RA1     //黄
#define GPIO_MOTOR_D    RA0     //橙
//===========================================================
//                           MACROS
//===========================================================
#define HAL_MCU_GIE()               GIE = 1
#define HAL_MCU_GID()               GIE = 0
#define ADC_IIR_FILTER(old, new)    (uint16_t)(((uint16_t)old*15+(uint16_t)new)>>4)
//===========================================================
//                      GLOBAL VARIABLES
//===========================================================
uint8_t speed;
uint8_t cnt;
uint8_t step;
uint8_t dir;         // 0 - forward
                     // 1 - backward
uint8_t pwr;         // 0 - stop
                     // 1 - run
uint8_t key_prev;
uint8_t key_curr;

uint8_t key_debounce;
uint16_t iir_adc_vr;

//Step 0    0    0    0    0
//          A    B    C    D
//Step 1    0    0    0    1
//Step 2    0    0    1    1
//Step 3    0    0    1    0
//Step 4    0    1    1    0
//Step 5    0    1    0    0
//Step 6    1    1    0    0
//Step 7    1    0    0    0
//Step 8    1    0    0    1

void interrupt ISR(void)
{
    if(T0IE&&T0IF)
    {
        T0IF = 0;
        
        if( pwr )
        {
            if( cnt++ >= speed )
            {
                cnt = 0;
                switch ( step & 0x07 )
                {
                    case 0: GPIO_MOTOR_A = 0; GPIO_MOTOR_B = 0; GPIO_MOTOR_C = 0; GPIO_MOTOR_D = 1; break;
                    case 1: GPIO_MOTOR_A = 0; GPIO_MOTOR_B = 0; GPIO_MOTOR_C = 1; GPIO_MOTOR_D = 1; break;
                    case 2: GPIO_MOTOR_A = 0; GPIO_MOTOR_B = 0; GPIO_MOTOR_C = 1; GPIO_MOTOR_D = 0; break;
                    case 3: GPIO_MOTOR_A = 0; GPIO_MOTOR_B = 1; GPIO_MOTOR_C = 1; GPIO_MOTOR_D = 0; break;
                    case 4: GPIO_MOTOR_A = 0; GPIO_MOTOR_B = 1; GPIO_MOTOR_C = 0; GPIO_MOTOR_D = 0; break;
                    case 5: GPIO_MOTOR_A = 1; GPIO_MOTOR_B = 1; GPIO_MOTOR_C = 0; GPIO_MOTOR_D = 0; break;
                    case 6: GPIO_MOTOR_A = 1; GPIO_MOTOR_B = 0; GPIO_MOTOR_C = 0; GPIO_MOTOR_D = 0; break;
                    case 7: GPIO_MOTOR_A = 1; GPIO_MOTOR_B = 0; GPIO_MOTOR_C = 0; GPIO_MOTOR_D = 1; break;
                }
                if( dir )
                    step++;
                else
                    step--;
            }
        }
        else
        {
            cnt = 0;
            step = 0;
            GPIO_MOTOR_A = 0; GPIO_MOTOR_B = 0; GPIO_MOTOR_C = 0; GPIO_MOTOR_D = 0;
        } 
    }
}

void device_init( void )
{
	OSCCON = 0B01110001;   // Bit7    >>>  LFMOD=0       >>> WDT振荡器频率=32KHz
                           // Bit6:4  >>>  IRCF[2:0]=101 >>> 内部RC频率=16MHz，需要设定为2T模式
                           // Bit0    >>>  SCS=1         >>> 系统时钟选择为内部振荡器
	MSCKCON= 0B00000000;   // Bit6    >>>  VREG_OE=0     >>> 禁止稳压输出
                           // Bit5    >>>  T2CKSRC=0     >>> Timer2时钟源为系统时钟
                           // Bit4    >>>  SLVREN=0      >>> 关闭LVR
	INTCON = 0B00000000;   // 暂禁止所有中断
	CMCON0 = 0B00000111;   // 关闭比较器，CxIN为数字IO引脚
	PORTA  = 0B10111100;   // RA0, RA1, RA6 默认输出0，经ULN2003反向后输出1，其他输出1
	TRISA  = 0B10111100;   // RA0, RA1, RA6 设置为输出，其他PORTA口设置为输入
	WPUA   = 0B10111100;   // RA0, RA1, RA6 禁用上拉，其他要开启上拉
	PORTC  = 0B11110111;   // PC3 默认输出0，经ULN2003反向后输出1，其他输出1
	TRISC  = 0B11110111;   // PC3 设置为输出，其他PC口设置为输入
	WPUC   = 0B11110001;   // PC1，PC2，PC3 关闭上拉，其他开启的内部弱上拉
	OPTION = 0B00000010;   // bit7=0, 开启PORTA内部上拉总闸，TIMER0预分频1/8
                           // Bit4=0, TIMER0 MODE
                           // PS=010, 1:8 TIMER0 RATE
                           // TIMER0溢出时间计算:
                           // 1. 单指令周期=(1/16MHz)*2 = 0.125us
                           // 2. TIMER0自加1所需时间=单指令周期*分频比=0.125us*8=1us
                           // 3. TIMER0溢出时间=(256-TIMER0初值)*自加1所需时间=(256-0)*1us=256us
    
    
    ANSEL = 0B01100000;    // 设置AN5(PC1),AN6(PC2)为模拟输入口
    ADCON1 = 0B01100000;   // DIVS=0,时钟选FOSC (16MHz)
                           // ADCS[2:0]=110,分频FOSC/64
    ADCON0 = 0B10000000;   // B7,ADFM=1,结果右对齐
                           // B6:5,VCFG=00,参考电压VDD
                           // B6:5,VCFG=01,参考电压内部2V
                           // B6:5,VCFG=10,参考电压内部3V
                           // B6:5,VCFG=11,参考电压Vref
                           // B4:2,CHS=000,选择AN0通道
                           // B1,GO,AD转换状态位
                           // B0,ADON=1,ADC使能
    
    T0IF = 0;              // 清TIMER0中断标志位
	T0IE = 1;              // 使能TIMER0中断
}

uint16_t hal_adc_read( uint8_t ch )
{
	uint16_t TempADCBuffer=0;
    //ch &= 0x07;
	ADCON0 = (ch<<2);         //设置通道
	ADCON0 |= 0b10000001;     //开启ADC电路
	__delay_us(20);           //等待采集到电压
	GO_DONE = 1;              //开启转换
	while(GO_DONE==1) CLRWDT();//等待转换完成
	TempADCBuffer = ADRESH;
	TempADCBuffer = (TempADCBuffer<<8)|ADRESL;
	ADON = 0;
	return(TempADCBuffer);
}

uint8_t get_speed( void )
{
    uint16_t adc_value;
    
    adc_value = hal_adc_read( ADC_CHANNEL_VR );
    iir_adc_vr = ADC_IIR_FILTER(iir_adc_vr, adc_value);
    adc_value = iir_adc_vr/20;
    adc_value += SPEED_MAX;
    
    return (uint8_t) adc_value;
}

uint8_t get_key( void )
{
    uint16_t adc_value;
    
    adc_value = hal_adc_read( ADC_CHANNEL_KEY );
    if( adc_value  < 512)
        return KEY_PWR;
    else if(adc_value < 900)
        return KEY_DIR;
    
    return 0x00;
}

main()
{
    device_init();
    
    pwr = 0;
    speed = 0;
    cnt = 0;
    step = 0;
    dir = 0;
    key_prev = 0x00;
    key_curr = 0x00;
    
    iir_adc_vr = hal_adc_read( ADC_CHANNEL_VR );
    speed = get_speed();
    
    HAL_MCU_GIE();
    while(1)
    {
        CLRWDT();
        speed = get_speed();
        
        key_curr = get_key();
        if( key_curr != key_prev )
        {
            key_prev = key_curr;
            key_debounce = 0;
        }
        else
        {
            if( key_debounce != KEY_DEBOUNCE )
            {
                key_debounce++;
                if( key_debounce == KEY_DEBOUNCE )
                {
                    if( key_curr == KEY_PWR )
                    {
                        if( pwr ) pwr = 0;
                        else      pwr = 1;
                    }
                    else if(  key_curr == KEY_DIR )
                    {
                        if( pwr )
                        {
                            if( dir ) dir = 0;
                            else      dir = 1;
                        }
                    }
                }
            }
        }
    }
}
//===========================================================
