#ifndef __BOARD_H__
#define __BOARD_H__

/*
 * This file contains definitions of board-specific variables and functions to support the
 * ArduinoPebbleSerial library.
 */

#include <Arduino.h>
#include <stdint.h>

// Helper macros for setting and clearing a bit within a register
#define cbi(sfr, bit) (sfr &= ~_BV(bit))
#define sbi(sfr, bit) (sfr |= _BV(bit))

// The board-specific variables are defined below
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__)
/* Arduino Mega, Teensy 2.0, etc */
#define BOARD_SERIAL Serial1
static inline void board_begin(void) {
}
static inline void board_set_tx_enabled(bool enabled) {
  if (enabled) {
    bitSet(UCSR1B, TXEN1);
    bitClear(UCSR1B, RXEN1);
  } else {
    bitClear(UCSR1B, TXEN1);
    bitClear(DDRD, 3);
    bitSet(PORTD, 3);
    bitSet(UCSR1B, RXEN1);
  }
}
static inline void board_set_even_parity(bool enabled) {
  if (enabled) {
    bitSet(UCSR1C, UPM11);
  } else {
    bitClear(UCSR1C, UPM11);
  }
}

#elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
/* Arduino Uno, etc */
#define BOARD_SERIAL Serial
static inline void board_begin(void) {
}
static inline void board_set_tx_enabled(bool enabled) {
  if (enabled) {
    bitSet(UCSR0B, TXEN0);
    bitClear(UCSR0B, RXEN0);
  } else {
    bitClear(UCSR0B, TXEN0);
    bitClear(DDRD, 1);
    bitSet(PORTD, 1);
    bitSet(UCSR0B, RXEN0);
  }
}
static inline void board_set_even_parity(bool enabled) {
  if (enabled) {
    bitSet(UCSR0C, UPM01);
  } else {
    bitClear(UCSR0C, UPM01);
  }
}
#elif defined(__MK20DX256__) || defined(__MK20DX128__)
/* Teensy 3.0, Teensy 3.1, etc */
#define BOARD_SERIAL Serial1
static inline void board_begin(void) {
  // configure TX as open-drain
  CORE_PIN1_CONFIG |= PORT_PCR_ODE;
}
static inline void board_set_tx_enabled(bool enabled) {
  // the TX and RX are tied together and we'll just drop any loopback frames
}
static inline void board_set_even_parity(bool enabled) {
  if (enabled) {
    serial_format(SERIAL_8E1);
  } else {
    serial_format(SERIAL_8N1);
  }
}


#elif defined(__SAMD21G18A__)
// as used in Arduino Zero
#define BOARD_SERIAL Serial1

static inline void board_begin(void) {
}

static inline void board_set_tx_enabled(bool enabled) {
  // the TX and RX are tied together and we'll just drop any loopback frames
  if (enabled) {
    SERCOM0->USART.CTRLB.bit.RXEN = 0x0u;
    // wait for bit to synchronize
    // or next register command will be ignored
    while(SERCOM0->USART.SYNCBUSY.bit.CTRLB);
    SERCOM0->USART.CTRLB.bit.TXEN = 0x1u;
    while(SERCOM0->USART.SYNCBUSY.bit.CTRLB);
  } else {
    SERCOM0->USART.CTRLB.bit.TXEN = 0x0u;
    while(SERCOM0->USART.SYNCBUSY.bit.CTRLB);
    SERCOM0->USART.CTRLB.bit.RXEN = 0x1u;
    while(SERCOM0->USART.SYNCBUSY.bit.CTRLB);
  }
}

// Parity Bit is locked, need to disable USART and set it again
//CTRLA: Bit24-27: 0x0 = no parity, 0x1 = parity, Bit 27 reserved
//CTRLB: Bit 13: 0x0 Even, 0x01 Odd
static inline void board_set_even_parity(bool enabled) {
  //Disable UART to unlock Control Registers
  SERCOM0->USART.CTRLA.bit.ENABLE = 0x0u;
  //Wait for then enable bit from SYNCBUSY is equal to 0;
  while(SERCOM0->USART.SYNCBUSY.bit.ENABLE);

  if (enabled) {
    //Setting the CTRLA register parity bit
    SERCOM0->USART.CTRLA.reg |=	SERCOM_USART_CTRLA_FORM(0x1u);
    //Setting the CTRLB register to even parity
    SERCOM0->USART.CTRLB.reg &=	~((0x1u) << SERCOM_USART_CTRLB_PMODE_Pos);
    while(SERCOM0->USART.SYNCBUSY.bit.CTRLB);
  } else {
    //Disabling parity bit in CTRLA register
    SERCOM0->USART.CTRLA.reg &=	~(SERCOM_USART_CTRLA_FORM(0x1u));
    //Setting the CTRLB register to even parity
    SERCOM0->USART.CTRLB.reg &=	~((0x1u) << SERCOM_USART_CTRLB_PMODE_Pos);
    while(SERCOM0->USART.SYNCBUSY.bit.CTRLB);
  }
  //SERCOM0->USART.INTFLAG.bit.TXC = 0x1u;
  //Enable UART again
  SERCOM0->USART.CTRLA.bit.ENABLE = 0x1u;
  //Wait for then enable bit from SYNCBUSY is equal to 0;
  while(SERCOM0->USART.SYNCBUSY.bit.ENABLE);
}



#else
#error "Board not supported!"
#endif

#endif // __BOARD_H__
