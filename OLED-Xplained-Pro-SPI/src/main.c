#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

// Configuracoes dos LEDS
#define LED1_PIO PIOA
#define LED1_PIO_ID ID_PIOA
#define LED1_PIO_IDX 0
#define LED1_PIO_IDX_MASK (1 << LED1_PIO_IDX)

#define LED2_PIO PIOC
#define LED2_PIO_ID ID_PIOC
#define LED2_PIO_IDX 30
#define LED2_PIO_IDX_MASK (1 << LED2_PIO_IDX)

#define LED3_PIO PIOB
#define LED3_PIO_ID ID_PIOB
#define LED3_PIO_IDX 2
#define LED3_PIO_IDX_MASK (1 << LED3_PIO_IDX)

// Configuracoes do BUT
#define BUT1_PIO PIOD
#define BUT1_PIO_ID 16
#define BUT1_PIO_IDX 28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX)

#define BUT2_PIO PIOC
#define BUT2_PIO_ID 12
#define BUT2_PIO_IDX 31
#define BUT2_PIO_IDX_MASK (1u << BUT2_PIO_IDX)

#define BUT3_PIO PIOA
#define BUT3_PIO_ID 10
#define BUT3_PIO_IDX 19
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX)
#define BUT_OLED_PRIORITY 5
typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t seccond;
} calendar;
volatile char flag_tc1 = 0;
volatile char flag_tc2 = 0;
volatile char flag_tc3 = 0;
volatile char rtt_flag;

volatile char but1_flag = 0;
volatile Bool f_rtt_alarme = false;
volatile char flag_rtc = 0;


void BUT1_CALLBACK(void);
void BUT2_CALLBACK(void);
void BUT3_CALLBACK(void);
void BUT1_CALLBACK(void)
{
	but1_flag = !but1_flag;
}
volatile char but2_flag = 0;

void BUT2_CALLBACK(void)
{
	but2_flag = !but2_flag;
}
volatile char but3_flag = 0;

void BUT3_CALLBACK(void)
{
	but3_flag = !but3_flag;
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);

void init(void);

void RTT_Handler(void)
{
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC)
	{
	}
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS)
	{
		rtt_flag = !rtt_flag;
		f_rtt_alarme = true;
	}
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);
static float get_time_rtt()
{
	uint ul_previous_time = rtt_read_timer_value(RTT);
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	rtt_write_alarm_time(RTT, IrqNPulses + ul_previous_time);
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}
void TC0_Handler(void)
{
	volatile uint32_t ul_dummy;
	ul_dummy = tc_get_status(TC0, 0);
	UNUSED(ul_dummy);
	flag_tc1 = !flag_tc1;
}
void TC3_Handler(void)
{
	volatile uint32_t ul_dummy;
	ul_dummy = tc_get_status(TC1, 0);
	UNUSED(ul_dummy);
	flag_tc2 = !flag_tc2;
}
void TC6_Handler(void)
{
	volatile uint32_t ul_dummy;
	ul_dummy = tc_get_status(TC2, 0);
	UNUSED(ul_dummy);
	flag_tc3 = !flag_tc3;
}
void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq)
{
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	pmc_enable_periph_clk(ID_TC);
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);
	NVIC_EnableIRQ((IRQn_Type)ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
	tc_start(TC, TC_CHANNEL);
}
void RTC_Handler(void){
	uint32_t ul_status = rtc_get_status(RTC);
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
		flag_rtc = 1;
	}
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type){
	pmc_enable_periph_clk(ID_RTC);
	rtc_set_hour_mode(rtc, 0);
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.seccond);
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 0);
	NVIC_EnableIRQ(id_rtc);
	rtc_enable_interrupt(rtc,  irq_type);
}
void init(void)
{

	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;

	pmc_enable_periph_clk(LED1_PIO_ID);
	pmc_enable_periph_clk(BUT1_PIO_ID);

	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 1, 0, 0);
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 0, 0, 0);

	pio_set_input(BUT1_PIO, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUT2_PIO, BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUT3_PIO, BUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_RISE_EDGE, BUT1_CALLBACK);
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);

	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, BUT_OLED_PRIORITY);

	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_FALL_EDGE, BUT2_CALLBACK);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);

	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, BUT_OLED_PRIORITY);

	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_RISE_EDGE, BUT3_CALLBACK);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);

	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, BUT_OLED_PRIORITY);
}

int main(void)
{
	init();
	board_init();
	sysclk_init();
	TC_init(TC0, ID_TC0, 0, 5);
	TC_init(TC1, ID_TC3, 0, 10);
	TC_init(TC2, ID_TC6, 0, 1);
	f_rtt_alarme = true;
	
	gfx_mono_ssd1306_init();
	rtt_flag =0;
	
	//gfx_mono_draw_string("5   10    1", 10, 16, &sysfont);
	calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_SECEN);
    rtc_set_date_alarm(RTC, 1, rtc_initial.month, 1, rtc_initial.day);
    rtc_set_time_alarm(RTC, 1, rtc_initial.hour, 1, rtc_initial.minute, 1, rtc_initial.seccond + 20);    
	
	
	while (1)
	{
		if(flag_rtc){
			rtc_get_time(RTC,&rtc_initial.hour,&rtc_initial.minute,&rtc_initial.seccond);
			char b[200];
			sprintf(b, "%2d : %2d : %2d",rtc_initial.hour,rtc_initial.minute,rtc_initial.seccond);
			gfx_mono_draw_string(b, 1,13, &sysfont);
			flag_rtc = 0;
		}	
		if (f_rtt_alarme)
		{
			uint16_t pllPreScale = (int)(((float)32768) / 4.0);
			uint32_t irqRTTvalue = 20;
			RTT_init(pllPreScale, irqRTTvalue);
			f_rtt_alarme = false;
		}else{pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);}
		if (!rtt_flag)
		{
			if (but1_flag)
			{
				if (flag_tc1)
				{
					pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
				}

				else
				{
					pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
				}
			}
			if (!but2_flag)
			{
				if (flag_tc2)
				{
					pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
				}
				else
				{
					pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
				}
			}
			if (but3_flag)
			{
				if (flag_tc3)
				{
					pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
				}
				else
				{
					pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
				}
			}
		}
	}
}
