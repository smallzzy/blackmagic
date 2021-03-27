#include "general.h"

#include <libopencm3/cm3/systick.h>

uint8_t running_status;
static volatile uint32_t time_ms;

void platform_timing_init(void)
{
	// todo: fix hardcoded value
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	/* Interrupt us at 10 Hz */
	systick_set_reload(2400000);
	systick_interrupt_enable();
	systick_counter_enable();
}

void sys_tick_handler(void)
{
	if(running_status)
		gpio_toggle(LED_PORT, LED_IDLE_RUN);

	time_ms += 100;

	uart_pop();
}

uint32_t platform_time_ms(void)
{
	return time_ms;
}
