// STM32 serial
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_SERIAL_BAUD
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/serial_irq.h" // serial_rx_byte
#include "command.h" // DECL_CONSTANT_STR
#include "gpio.h" // gpio_out_setup
#include "internal.h" // enable_pclock
#include "sched.h" // DECL_INIT

// Select the configured serial port
#if CONFIG_STM32_SERIAL_USART1
  DECL_CONSTANT_STR("RESERVE_PINS_serial", "PA10,PA9");
  #define GPIO_Rx GPIO('A', 10)
  #define GPIO_Tx GPIO('A', 9)
  #define GPIO_AF_MODE 7
  #define USARTx USART1
  #define USARTx_IRQn USART1_IRQn
#elif CONFIG_STM32_SERIAL_USART1_ALT_PB7_PB6
  DECL_CONSTANT_STR("RESERVE_PINS_serial", "PB7,PB6");
  #define GPIO_Rx GPIO('B', 7)
  #define GPIO_Tx GPIO('B', 6)
  #define GPIO_AF_MODE 7
  #define USARTx USART1
  #define USARTx_IRQn USART1_IRQn
#elif CONFIG_STM32_SERIAL_USART2
  DECL_CONSTANT_STR("RESERVE_PINS_serial", "PA3,PA2");
  #define GPIO_Rx GPIO('A', 3)
  #define GPIO_Tx GPIO('A', 2)
  #define GPIO_AF_MODE 7
  #define USARTx USART2
  #define USARTx_IRQn USART2_IRQn
#elif CONFIG_STM32_SERIAL_USART2_ALT_PD6_PD5
  DECL_CONSTANT_STR("RESERVE_PINS_serial", "PD6,PD5");
  #define GPIO_Rx GPIO('D', 6)
  #define GPIO_Tx GPIO('D', 5)
  #define GPIO_AF_MODE 7
  #define USARTx USART2
  #define USARTx_IRQn USART2_IRQn
#elif CONFIG_STM32_SERIAL_USART3
  DECL_CONSTANT_STR("RESERVE_PINS_serial", "PB11,PB10");
  #define GPIO_Rx GPIO('B', 11)
  #define GPIO_Tx GPIO('B', 10)
  #define GPIO_AF_MODE 7
  #define USARTx USART3
  #define USARTx_IRQn USART3_IRQn
#elif CONFIG_STM32_SERIAL_USART3_ALT_PD9_PD8
  DECL_CONSTANT_STR("RESERVE_PINS_serial", "PD9,PD8");
  #define GPIO_Rx GPIO('D', 9)
  #define GPIO_Tx GPIO('D', 8)
  #define GPIO_AF_MODE 7
  #define USARTx USART3
  #define USARTx_IRQn USART3_IRQn
#elif CONFIG_STM32_SERIAL_USART6
  DECL_CONSTANT_STR("RESERVE_PINS_serial", "PA12,PA11");
  #define GPIO_Rx GPIO('A', 12)
  #define GPIO_Tx GPIO('A', 11)
  #define GPIO_AF_MODE 8
  #define USARTx USART6
  #define USARTx_IRQn USART6_IRQn
#elif CONFIG_STM32_SERIAL_USART6_ALT_PC7_PC6
  DECL_CONSTANT_STR("RESERVE_PINS_serial", "PC7,PC6");
  #define GPIO_Rx GPIO('C', 7)
  #define GPIO_Tx GPIO('C', 6)
  #define GPIO_AF_MODE 8
  #define USARTx USART6
  #define USARTx_IRQn USART6_IRQn
#endif

#define CR1_FLAGS (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE   \
                   | USART_CR1_RXNEIE)

#if CONFIG_STM32_FLASHFORGE_IFS
  #include "board/irq.h" // irq_save
  #include "board/misc.h" // timer_read_time
  // Flashforge AD5X IFS: RS485 transceiver direction on PA11
  // (high = transmit, low = receive)
  DECL_CONSTANT_STR("RESERVE_PINS_rs485", "PA11");
  DECL_CONSTANT("SERIAL_HALF_DUPLEX", 1);
  #define GPIO_RS485_DIR GPIO('A', 11)
  // Only start sending 1-2ms after the last received byte: the host
  // releases the bus 1ms after its last byte and polls with sync bytes
  #define RS485_GAP_US 1000
  #define RS485_WINDOW_US 2000
  static struct gpio_out rs485_dir;
  static uint32_t rs485_last_rx;
  static uint8_t rs485_tx_pending, rs485_timer_armed;
  static struct timer rs485_timer;

static uint_fast8_t
rs485_event(struct timer *t)
{
    uint32_t since = timer_read_time() - rs485_last_rx;
    if (since < timer_from_us(RS485_GAP_US)) {
        t->waketime = rs485_last_rx + timer_from_us(RS485_GAP_US);
        return SF_RESCHEDULE;
    }
    rs485_timer_armed = 0;
    if (since <= timer_from_us(RS485_WINDOW_US)) {
        rs485_tx_pending = 0;
        gpio_out_write(rs485_dir, 1);
        USARTx->CR1 = CR1_FLAGS | USART_CR1_TXEIE;
    }
    return SF_DONE;
}

// Must be called with irqs disabled
static void
rs485_arm(void)
{
    if (rs485_timer_armed)
        return;
    uint32_t waketime = rs485_last_rx + timer_from_us(RS485_GAP_US);
    uint32_t now = timer_read_time() + timer_from_us(10);
    if (timer_is_before(waketime, now))
        waketime = now;
    rs485_timer_armed = 1;
    rs485_timer.func = rs485_event;
    rs485_timer.waketime = waketime;
    sched_add_timer(&rs485_timer);
}

void
rs485_shutdown(void)
{
    rs485_timer_armed = 0;
    if (rs485_tx_pending)
        rs485_arm();
}
DECL_SHUTDOWN(rs485_shutdown);
#endif

void
USARTx_IRQHandler(void)
{
    uint32_t sr = USARTx->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        // The ORE flag is automatically cleared by reading SR, followed
        // by reading DR.
        serial_rx_byte(USARTx->DR);
#if CONFIG_STM32_FLASHFORGE_IFS
        rs485_last_rx = timer_read_time();
        if (rs485_tx_pending) {
            irqstatus_t flag = irq_save();
            rs485_arm();
            irq_restore(flag);
        }
#endif
    }
    if (sr & USART_SR_TXE && USARTx->CR1 & USART_CR1_TXEIE) {
        uint8_t data;
        int ret = serial_get_tx_byte(&data);
        if (ret)
            // RS485: wait for the last byte to leave the shift register
            USARTx->CR1 = CR1_FLAGS | (CONFIG_STM32_FLASHFORGE_IFS
                                       ? USART_CR1_TCIE : 0);
        else
            USARTx->DR = data;
    }
#if CONFIG_STM32_FLASHFORGE_IFS
    if (sr & USART_SR_TC && USARTx->CR1 & USART_CR1_TCIE) {
        USARTx->CR1 = CR1_FLAGS;
        gpio_out_write(rs485_dir, 0);
    }
#endif
}

void
serial_enable_tx_irq(void)
{
#if CONFIG_STM32_FLASHFORGE_IFS
    irqstatus_t flag = irq_save();
    if (USARTx->CR1 & (USART_CR1_TXEIE | USART_CR1_TCIE)) {
        // burst in progress, keep it going
        USARTx->CR1 = CR1_FLAGS | USART_CR1_TXEIE;
    } else if (!rs485_tx_pending) {
        rs485_tx_pending = 1;
        rs485_arm();
    }
    irq_restore(flag);
#else
    USARTx->CR1 = CR1_FLAGS | USART_CR1_TXEIE;
#endif
}

void
serial_init(void)
{
    enable_pclock((uint32_t)USARTx);

    uint32_t pclk = get_pclock_frequency((uint32_t)USARTx);
    uint32_t div = DIV_ROUND_CLOSEST(pclk, CONFIG_SERIAL_BAUD);
    USARTx->BRR = (((div / 16) << USART_BRR_DIV_Mantissa_Pos)
                   | ((div % 16) << USART_BRR_DIV_Fraction_Pos));
#if CONFIG_STM32_FLASHFORGE_IFS
    // undo the bootloader's usart setup
    USARTx->CR2 = 0;
    USARTx->CR3 = 0;
#endif
    USARTx->CR1 = CR1_FLAGS;
    armcm_enable_irq(USARTx_IRQHandler, USARTx_IRQn, 0);

    gpio_peripheral(GPIO_Rx, GPIO_FUNCTION(GPIO_AF_MODE), 1);
    gpio_peripheral(GPIO_Tx, GPIO_FUNCTION(GPIO_AF_MODE), 0);
#if CONFIG_STM32_FLASHFORGE_IFS
    rs485_dir = gpio_out_setup(GPIO_RS485_DIR, 0);
#endif
}
DECL_INIT(serial_init);
