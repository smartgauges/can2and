#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/can.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "ring.h"

#define TICK_HZ 1000
#define SEC_TO_TICK(SEC) (TICK_HZ * SEC)
#define MSEC_TO_TICK(MSEC) ((MSEC * TICK_HZ)/1000)

#define LED_PORT GPIOC
#define LED_PIN GPIO13
static uint16_t led_div = 0;
void led_on (void)
{
	gpio_clear(LED_PORT, LED_PIN);
	led_div = 1;
}

void led_off (void)
{
	gpio_set(LED_PORT, LED_PIN);
	led_div = 0;
}

uint8_t led_is_on (void)
{
	return led_div;
}

void led_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOC);

	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_PIN);

	/* Switch off LED. */
	led_off();
}

void led_tick(void)
{
	if (led_is_on()) {

		if (led_div++ >= MSEC_TO_TICK(50)) {

			led_off();
		}
	}
}

static void systick_setup(void)
{
	/* 72MHz / 8 => 6000000 counts per second */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	/* clear counter so it starts right away */
	STK_CVR = 0;

	systick_set_reload(9000000 / TICK_HZ);

	systick_interrupt_enable();

	/* Start counting. */
	systick_counter_enable();

}

typedef struct
{
	volatile uint16_t flag_tick;
	volatile uint16_t flag_50ms;
	volatile uint16_t flag_250ms;
	volatile uint16_t flag_1000ms;
} tick_t;
volatile tick_t timer = { 0, 0, 0, 0 };

void sys_tick_handler(void)
{
	static uint16_t div_1000ms = 0;
	static uint16_t div_250ms = 0;
	static uint16_t div_50ms = 0;

	timer.flag_tick = 1;

	if (++div_1000ms >= SEC_TO_TICK(1)) {

		div_1000ms = 0;
		timer.flag_1000ms = 1;
	}

	if (++div_250ms >= MSEC_TO_TICK(250)) {

		div_250ms = 0;
		timer.flag_250ms = 1;
	}

	if (++div_50ms >= MSEC_TO_TICK(100)) {

		div_50ms = 0;
		timer.flag_50ms = 1;
	}

	led_tick();
}

void setup_usart(uint32_t speed)
{
	usart_disable(USART1);

	/* Enable clocks for USART1. */
	rcc_periph_clock_enable(RCC_USART1);

	rcc_periph_clock_enable(RCC_GPIOA);

	/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART1_IRQ);

	gpio_set_mode(GPIO_BANK_USART1_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	gpio_set_mode(GPIO_BANK_USART1_RX, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, speed);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX);

	/* Enable USART1 Receive interrupt. */
	USART_CR1(USART1) |= USART_CR1_RXNEIE;

	/* Finally enable the USART. */
	usart_enable(USART1);
}

static struct ring tx_ring;
static uint8_t tx_ring_buffer[500];
static struct ring rx_ring;
static uint8_t rx_ring_buffer[500];

void usart1_isr(void)
{
	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

		/* Retrieve the data from the peripheral. */
		ring_write_ch(&rx_ring, usart_recv(USART1));
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_TXE) != 0)) {

		uint8_t ch;
		if (!ring_read_ch(&tx_ring, &ch)) {

			/* Disable the TXE interrupt, it's no longer needed. */
			USART_CR1(USART1) &= ~USART_CR1_TXEIE;
		} else {

			/* Put data into the transmit register. */
			usart_send(USART1, ch);
		}
	}
}

int usart_write(const uint8_t * ptr, int len)
{
	int ret = ring_write(&tx_ring, (uint8_t *)ptr, len);

	USART_CR1(USART1) |= USART_CR1_TXEIE;

	return ret;
}

typedef enum e_speed_t
{
	e_speed_125 = 0,
	e_speed_250,
	e_speed_500,
	e_speed_1000,
	e_speed_nums
} e_speed_t;

typedef struct speed_t
{
	uint32_t sjw;
	uint32_t ts1;
	uint32_t ts2;
	uint32_t brp;
} speed_t;

/* APB1 36 MHz */
static speed_t speeds[e_speed_nums] = 
{
	{ CAN_BTR_SJW_1TQ, CAN_BTR_TS1_13TQ, CAN_BTR_TS2_2TQ, 18 },
	{ CAN_BTR_SJW_1TQ, CAN_BTR_TS1_13TQ, CAN_BTR_TS2_2TQ, 9 },
	{ CAN_BTR_SJW_1TQ, CAN_BTR_TS1_15TQ, CAN_BTR_TS2_2TQ, 4 },
	{ CAN_BTR_SJW_1TQ, CAN_BTR_TS1_15TQ, CAN_BTR_TS2_2TQ, 2 },
};

uint8_t can_set_speed(e_speed_t speed)
{
	nvic_disable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
	can_disable_irq(CAN1, CAN_IER_FMPIE0);

	/* Reset CAN. */
	can_reset(CAN1);

	/* CAN cell init. apb1 48 MHZ */
	int ret = can_init(CAN1,
		     false,           /* TTCM: Time triggered comm mode? */
		     true,            /* ABOM: Automatic bus-off management? */
		     false,           /* AWUM: Automatic wakeup mode? */
		     false,           /* NART: No automatic retransmission? */
		     false,           /* RFLM: Receive FIFO locked mode? */
		     false,           /* TXFP: Transmit FIFO priority? */
		     speeds[speed].sjw,
		     speeds[speed].ts1,
		     speeds[speed].ts2,
		     speeds[speed].brp,
		     false,
		     false
		     );

	if (ret)
		return ret;

	/* CAN filter 0 init. */
	can_filter_id_mask_32bit_init(CAN1,
				0,     /* Filter ID */
				0,     /* CAN ID */
				0,     /* CAN ID mask */
				0,     /* FIFO assignment (here: FIFO0) */
				true); /* Enable the filter. */

	/* Enable CAN RX interrupt. */
	can_enable_irq(CAN1, CAN_IER_FMPIE0);
	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);

	return 0;
}

enum e_can_types
{
	e_can_simple = 0x0,
	e_can_statistic = 0x1,
	e_can_odd = 0x2,
	e_can_ext = 0x40,
	e_can_rtr = 0x80,
};

typedef struct msg_can_t
{
	uint32_t id;
	uint32_t num;
	uint8_t type;
	uint8_t len;
	uint8_t data[8];
} __attribute__ ((__packed__)) msg_can_t;

#define MSGS_SIZE 80
struct msg_can_t msgs[MSGS_SIZE];
static uint8_t msgs_size = 0;

static uint8_t can_setup(void)
{
	/* Enable peripheral clocks. */
	rcc_periph_clock_enable(RCC_CAN);

	AFIO_MAPR |= AFIO_MAPR_CAN1_REMAP_PORTB;

	/* Configure CAN pin: RX (input pull-up). */
	gpio_set_mode(GPIO_BANK_CAN1_PB_RX, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN1_PB_RX);
	gpio_set(GPIO_BANK_CAN1_PB_RX, GPIO_CAN1_PB_RX);

	/* Configure CAN pin: TX. */
	gpio_set_mode(GPIO_BANK_CAN1_PB_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN1_PB_TX);

	/* NVIC setup. */
	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
	nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 1);

	return can_set_speed(e_speed_125);
}

uint8_t can_get_msgs_num(void)
{
	return msgs_size;
}

uint8_t can_get_msg(struct msg_can_t * msg, uint8_t idx)
{
	if (idx >= msgs_size)
		return 0;

	*msg = msgs[idx];

	return 1;
}

void usb_lp_can_rx0_isr(void)
{
	uint32_t fmi;
	struct msg_can_t msg;
	uint8_t i, j;

	bool rtr = 0, ext = 0;
	can_receive(CAN1, 0, false, &msg.id, &ext, &rtr, &fmi, &msg.len, msg.data);

	msg.type = 0;
	if (rtr)
		msg.type |= e_can_rtr;
	if (ext)
		msg.type |= e_can_ext;

	uint8_t found = 0;
	for (i = 0; i < msgs_size; i++) {

		if (msgs[i].id == msg.id) {

			msgs[i].len = msg.len;
			for (j = 0; j < 8; j++)
				msgs[i].data[j] = msg.data[j];
			msgs[i].num++;
			found = 1;
			break;
		}
	}

	if (!found && msgs_size < MSGS_SIZE) {

		msgs[msgs_size] = msg;
		msgs[msgs_size].num = 1;
		msgs_size++;
	}

	can_fifo_release(CAN1, 0);
}

void can_snd_msg(struct msg_can_t * msg)
{
	if (!can_available_mailbox(CAN1)) {

		CAN_TSR(CAN1) |= CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_TABRQ2;
	}

	bool rtr = msg->type & e_can_rtr;
	bool ext = msg->type & e_can_ext;
	can_transmit(CAN1, msg->id, ext, rtr, msg->len, msg->data);
}

static uint8_t radar_checksum(uint8_t * buf, uint8_t len)
{
	uint8_t sum = 0;
	for (uint8_t i = 0; i < len; i++)
		sum += buf[i];

	sum = sum ^ 0xff;

	return sum;
}

static uint8_t radar_scale(uint32_t value, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
	return (((value - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min;
}

//#define LR2_2007
void radar_process(void)
{
	uint8_t msgs_num = can_get_msgs_num();

	if (!msgs_num)
		return;

	/*
	 * front
	 * 2E23040810182088
	 *
	 * rear
	 * 2E22040810182089
	 */
	struct msg_can_t msg;
	for (uint8_t i = 0; i < msgs_num; i++) {

		if (!can_get_msg(&msg, i))
			continue;
#ifdef LR2_2007
		if (0x188 != msg.id)
			continue;

		if (0x70 != msg.data[0])
			continue;

		uint32_t f = (msg.data[5] << 16) | (msg.data[6] << 8) | msg.data[7];
		uint32_t b = (msg.data[2] << 16) | (msg.data[3] << 8) | msg.data[4];
#else
		if (0x4a6 != msg.id)
			continue;

		if (!(msg.data[0] & 0x04))
			continue;

		uint32_t f = (msg.data[2] << 16) | (msg.data[3] << 8) | msg.data[4];
		uint32_t b = (msg.data[5] << 16) | (msg.data[6] << 8) | msg.data[7];
#endif
		uint8_t f0 = (f >> 15) & 0x1f;
		uint8_t f1 = (f >> 10) & 0x1f;
		uint8_t f2 = (f >> 5) & 0x1f;
		uint8_t f3 = f & 0x1f;

		uint8_t fbuf[] = { 0x2e, 0x23, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00 };
		fbuf[3] = 60 - radar_scale(f3, 0, 32, 0, 60);
		fbuf[4] = 120 - radar_scale(f2, 0, 32, 0, 120);
		fbuf[5] = 120 - radar_scale(f1, 0, 32, 0, 120);
		fbuf[6] = 60 - radar_scale(f0, 0, 32, 0, 60);
		fbuf[7] = radar_checksum(fbuf + 1, 6);

		uint8_t b0 = (b >> 15) & 0x1f;
		uint8_t b1 = (b >> 10) & 0x1f;
		uint8_t b2 = (b >> 5) & 0x1f;
		uint8_t b3 = b & 0x1f;

		uint8_t rbuf[] = { 0x2e, 0x22, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00 };
		rbuf[3] = 60 - radar_scale(b3, 0, 32, 0, 60);
		rbuf[4] = 165 - radar_scale(b2, 0, 32, 0, 165);
		rbuf[5] = 165 - radar_scale(b1, 0, 32, 0, 165);
		rbuf[6] = 60 - radar_scale(b0, 0, 32, 0, 60);
		rbuf[7] = radar_checksum(rbuf + 1, 6);

		usart_write(fbuf, sizeof(fbuf));
		usart_write(rbuf, sizeof(rbuf));
	}
}

#define sei() __asm__ __volatile__ ("cpsie i")
#define cli() __asm__ __volatile__ ("cpsid i")

int main(void)
{
	cli();

	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	systick_setup();
	led_setup();

	/* Enable AFIO clock. */
	rcc_periph_clock_enable(RCC_AFIO);
	can_setup();

	ring_init(&rx_ring, rx_ring_buffer, sizeof(rx_ring_buffer));
	ring_init(&tx_ring, tx_ring_buffer, sizeof(tx_ring_buffer));
	setup_usart(38400);

	sei();

	while(1)
	{
		if (timer.flag_tick) {

			timer.flag_tick = 0;

			if (timer.flag_250ms) {

				timer.flag_250ms = 0;
				radar_process();
			}

			if (timer.flag_1000ms) {

				timer.flag_1000ms = 0;

				led_on();
			}
		}
	}

	return 0;
}

