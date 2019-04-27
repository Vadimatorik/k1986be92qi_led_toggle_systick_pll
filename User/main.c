#include "1986be9x_config.h"
#include "1986BE9x.h"
#include "1986BE9x_uart.h"
#include "1986BE9x_port.h"
#include "1986BE9x_rst_clk.h"
#include "1986BE9x_it.h"
#include "mlt_lcd.h"
#include "MilFlash.h"

void Led_init (void)
{
  RST_CLK->PER_CLOCK |=(1<<23);                   //Включаем тактирование порта C.
	PORTC->OE |= 1;                                 //Порт - выход.
	PORTC->ANALOG |= 1;                             //Порт - цифоровой. 
	PORTC->PWR |= 1;                                //Порт - медленный режим.
}

#define CLKSOURCE (1<<2)                          //Указывает источник синхросигнала: 0 - LSI, 1 - HCLK.
#define TCKINT    (1<<1)                          //Разрешает запрос на прерывание от системного таймера.
#define ENABLE    (1<<0)                          //Разрешает работу таймера.
void Init_SysTick (void)                          //Прерывание раз в милли секунду. 
{
	SysTick->LOAD = (8000000/1000)-1;              
	SysTick->CTRL |= CLKSOURCE|TCKINT|ENABLE;
}

volatile uint32_t Delay_dec = 0;
void SysTick_Handler (void)
{
	if (Delay_dec) Delay_dec--;
}

void Delay_ms (uint32_t Delay_ms_Data)
{
	Delay_dec = Delay_ms_Data;
	while (Delay_dec) {};
}

#define HCLK_SEL(CPU_C3)       (1<<8)
#define CPU_C1_SEL(HSE)        (1<<1)
#define CPU_C2_SEL(CPU_C2_SEL) (1<<2)
#define PCLK_EN(RST_CLK)       (1<<4)
#define HS_CONTROL(HSE_ON)     (1<<0)
#define REG_0F(HSI_ON)        ~(1<<22)
#define RTC_CS(ALRF)           (1<<2)
#define PCLK(BKP)              (1<<27)

#define RST_CLK_ON_Clock()       RST_CLK->PER_CLOCK |= PCLK_EN(RST_CLK)                 //Включаем тактирование контроллера тактовой частоты (по умолчанию включено).
#define HSE_Clock_ON()           RST_CLK->HS_CONTROL = HS_CONTROL(HSE_ON)               //Разрешаем использование HSE генератора. 
#define HSE_Clock_OffPLL()       RST_CLK->CPU_CLOCK  = CPU_C1_SEL(HSE)|HCLK_SEL(CPU_C3);//Настраиваем "путь" сигнала и включаем тактирование от HSE генератора.

#define PLL_CONTROL_PLL_CPU_ON  (1<<2)                                                  //PLL включена. 
#define PLL_CONTROL_PLL_CPU_PLD (1<<3)                                                  //Бит перезапуска PLL.
void HSE_PLL (uint8_t PLL_multiply)                                                              //Сюда передаем частоту в разах "в 2 раза" например. 
{
	RST_CLK->PLL_CONTROL  = RST_CLK->PLL_CONTROL&(~(0xF<<8));                                      //Удаляем старое значение.
	RST_CLK->PLL_CONTROL |= PLL_CONTROL_PLL_CPU_ON|((PLL_multiply-1)<<8)|PLL_CONTROL_PLL_CPU_PLD;  //Включаем PLL и включаем умножение в X раз, а так же перезапускаем PLL.
	RST_CLK->CPU_CLOCK   |= HCLK_SEL(CPU_C3)|CPU_C2_SEL(CPU_C2_SEL)|CPU_C1_SEL(HSE);               //Настриваем "маршрут" частоты через PLL и включаем тактирование от HSE.
}

void Block (void)                                //Подпрограмма ожидания (защиты).
{
	PORTC->RXTX |= 1;
	Delay_ms (1000);
	PORTC->RXTX = 0;
	Delay_ms (1000);
}

int main (void)
{
	Init_SysTick();                                 //Инициализируем системный таймер. 
	Led_init();                                     //Инициализируем ножку 0 порта C для светодиода. 
	Block();                                        //Подпрограмма ожидания (защиты).
	HSE_Clock_ON();                                 //Разрешаем использование HSE генератора. 
	HSE_PLL(2);                                     //Включаем тактирование с умножением 2
	
	uint8_t PLL_Data = 1;                           //Здесь храним коэффициент умножения. 
  while (1)
	{
		PORTC->RXTX |= 1;
		Delay_ms (1000);
		PORTC->RXTX = 0;
	  Delay_ms (1000);
		if (PLL_Data<10) PLL_Data++; else PLL_Data=1; //Если не перешли максимум - умножаем еще. Перешли - с начала.
    HSE_PLL(PLL_Data); 
	}
}
