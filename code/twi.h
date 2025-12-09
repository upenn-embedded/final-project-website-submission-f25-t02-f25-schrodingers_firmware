#ifndef TWI_H
#define TWI_H
#include <stdint.h>

#ifndef F_CPU
#define F_CPU 16000000UL      // 16 MHz CPU clock for ATmega328PB
#endif

#define F_SCL     100000UL    // 100 kHz I2C
#define PRESCALER 1
#define TWBR_VALUE ((((F_CPU / F_SCL) / PRESCALER) - 16) / 2)

void    TWI0_init(void);
void    TWI0_reset(void);  // Reset TWI bus
uint8_t TWI0_start(uint8_t address);
void    TWI0_stop(void);
uint8_t TWI0_write(uint8_t data);
uint8_t TWI0_read_ack(void);
uint8_t TWI0_read_nack(void);

#endif