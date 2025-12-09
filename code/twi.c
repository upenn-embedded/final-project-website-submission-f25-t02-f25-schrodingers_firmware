#include "twi.h"
#include <avr/io.h>

#define TWI_TIMEOUT 5000  // Timeout counter (~50ms at 100kHz)

void TWI0_init(void)
{
    // Disable TWI first
    TWCR0 = 0;
    
    // Prescaler = 1
    TWSR0 &= ~((1 << TWPS1) | (1 << TWPS0));
    // Set SCL frequency
    TWBR0 = TWBR_VALUE;
    // Enable TWI
    TWCR0 = (1 << TWEN);
}

void TWI0_reset(void)
{
    // Full TWI reset
    TWCR0 = 0;  // Disable TWI
    TWSR0 = 0;  // Clear status
    TWBR0 = 0;  // Clear bit rate
    TWDR0 = 0xFF;  // Reset data register
    
    // Re-initialize
    TWI0_init();
}

uint8_t TWI0_start(uint8_t address)
{
    uint16_t timeout;
    
    // Send START
    TWCR0 = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    timeout = TWI_TIMEOUT;
    while (!(TWCR0 & (1 << TWINT))) {
        if (--timeout == 0) return 0;  // Timeout
    }
    
    // Check status
    uint8_t status = TWSR0 & 0xF8;
    if (status != 0x08 && status != 0x10) return 0;  // START or repeated START failed
    
    // Send address
    TWDR0 = address;
    TWCR0 = (1 << TWINT) | (1 << TWEN);
    timeout = TWI_TIMEOUT;
    while (!(TWCR0 & (1 << TWINT))) {
        if (--timeout == 0) return 0;  // Timeout
    }
    
    // Check ACK
    status = TWSR0 & 0xF8;
    if (status != 0x18 && status != 0x40) return 0;  // Address not ACKed
    
    return 1;  // Success
}

void TWI0_stop(void)
{
    TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    // Wait for STOP to complete
    uint16_t timeout = TWI_TIMEOUT;
    while ((TWCR0 & (1 << TWSTO)) && --timeout);
}

uint8_t TWI0_write(uint8_t data)
{
    uint16_t timeout;
    
    TWDR0 = data;
    TWCR0 = (1 << TWINT) | (1 << TWEN);
    timeout = TWI_TIMEOUT;
    while (!(TWCR0 & (1 << TWINT))) {
        if (--timeout == 0) return 0;  // Timeout
    }
    
    // Check ACK
    uint8_t status = TWSR0 & 0xF8;
    if (status != 0x28) return 0;  // Data not ACKed
    
    return 1;  // Success
}

uint8_t TWI0_read_ack(void)
{
    uint16_t timeout;
    
    TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    timeout = TWI_TIMEOUT;
    while (!(TWCR0 & (1 << TWINT))) {
        if (--timeout == 0) return 0xFF;  // Timeout, return dummy
    }
    return TWDR0;
}

uint8_t TWI0_read_nack(void)
{
    uint16_t timeout;
    
    TWCR0 = (1 << TWINT) | (1 << TWEN);
    timeout = TWI_TIMEOUT;
    while (!(TWCR0 & (1 << TWINT))) {
        if (--timeout == 0) return 0xFF;  // Timeout, return dummy
    }
    return TWDR0;
}