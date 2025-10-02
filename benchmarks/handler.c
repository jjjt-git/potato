#include"platform.h"
#include"uart.h"

static struct uart out;

void exception_handler(uint32_t cause, void* epc, void* regbase) {

	uart_initialize(&out, (volatile void*) PLATFORM_UART0_BASE);
	uart_set_divisor(&out, uart_baud2divisor(19200, PLATFORM_SYSCLK_FREQ));

	if ((cause >> 3) == 1) { // ecall
		return;
		uart_tx_string(&out, "\n------------------------");
		int mode;
		asm volatile("csrr %0, 0xBFF" : "=r" (mode) : : );
		mode += 3;
		if ((3 & (mode >> 1)) == 0) { // end execution
			uart_tx_string(&out, "runs finished");
			uart_tx_string(&out, "------------------------\n");
			while (1) asm volatile ("wfi" : : : );
		} else {
			while (uart_tx_fifo_full(&out)); uart_tx(&out, '0' + (3 & (mode >> 1)));
			uart_tx_string(&out, " is the current mode\n------------------------\n");
			asm volatile("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;csrw 0xBFF, %0;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop" : : "r" (mode) : );

			asm volatile("csrw mtvec, x0" : : : ); // set jump to start
		}
	} else {
		while (uart_tx_fifo_full(&out));
		uart_tx(&out, 'E');
	}
}
