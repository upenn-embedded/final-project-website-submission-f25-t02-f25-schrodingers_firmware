/*
 * ============================================================================
 * BrakeBright: Intelligent Bicycle Brake Light Firmware
 * ============================================================================
 * 
 * PROJECT OVERVIEW:
 * This firmware implements an intelligent bicycle brake light system that:
 * 1. Automatically detects braking using an accelerometer (IMU)
 * 2. Adjusts LED baseline brightness based on ambient light conditions
 * 3. Flashes the LED at full brightness when braking is detected
 * 4. Smoothly fades the LED back to baseline after braking stops
 * 
 * HARDWARE COMPONENTS:
 * - ATmega328PB Microcontroller (running at 16 MHz)
 * - LSM6DS0 IMU (3-axis accelerometer + gyroscope) via I2C
 * - Photoresistor (LDR) connected to ADC0 for ambient light sensing
 * - High-brightness red LED driven by PWM through a MOSFET
 * - UART for debugging and status output
 * 
 * KEY FEATURES:
 * - Real-time brake detection with noise filtering
 * - Ambient light compensation (dimmer at night, brighter in daylight)
 * - Smooth LED transitions for professional appearance
 * - Non-blocking state machine for responsive operation
 * - Comprehensive UART logging for debugging
 */


#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "lsm6ds0.h"
#include "twi.h"

/* ========================= UART ========================= */

static void uart_init(uint32_t baud)
{
    uint16_t ubrr = (F_CPU / (16UL * baud)) - 1;
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart_putc(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

static void uart_print(const char *s)
{
    while (*s) uart_putc(*s++);
}

static void uart_println(const char *s)
{
    uart_print(s);
    uart_putc('\r');
    uart_putc('\n');
}

static void uart_print_int(int32_t v)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%ld", (long)v);
    uart_print(buf);
}

static void uart_print_float(float f)
{
    char buf[20];
    snprintf(buf, sizeof(buf), "%.3f", (double)f);
    uart_print(buf);
}

/* ========================= Timer (millis) ========================= */

static volatile uint32_t ms_counter = 0;

ISR(TIMER0_COMPA_vect)
{
    ms_counter++;
}

static void timebase_init(void)
{
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A  = 249;
    TIMSK0 |= (1 << OCIE0A);
}

static uint32_t millis(void)
{
    uint32_t m;
    uint8_t sreg = SREG;
    cli();
    m = ms_counter;
    SREG = sreg;
    return m;
}

/* ========================= PWM ========================= */

static void pwm_init(void)
{
    DDRB |= (1 << DDB1);
    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << WGM12)  | (1 << CS11) | (1 << CS10);
    OCR1A  = 0;
}

static void pwm_set_duty(uint8_t duty)
{
    OCR1A = duty;
}

/* ========================= Ambient Light (ADC) ========================= */

static void ambient_init(void)
{
    DDRC &= ~(1 << DDC0);          // ADC0 input (PC0)
    ADMUX  = (1 << REFS0);         // AVcc reference, ADC0 channel
    ADCSRA = (1 << ADEN) |
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler 128
}

static uint16_t read_adc0(void)
{
    ADMUX &= 0xF0;   // select ADC0
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

/*
 * Map LDR ADC reading (0–1023) to baseline LED duty (0–255).
 * Dark room -> dim baseline
 * Bright light -> brighter baseline (but still much less than brake)
 */
static uint8_t ambient_compute_duty(void)
{
    uint16_t adc = read_adc0(); // 0..1023

    // Tunable points:
    const uint16_t dark_adc   = 200;  // below this: "dark"
    const uint16_t bright_adc = 800;  // above this: "bright"

    const uint8_t dark_duty   = 5;    // very dim in dark
    const uint8_t bright_duty = 50;   // brighter in daylight

    if (adc <= dark_adc) return dark_duty;
    if (adc >= bright_adc) return bright_duty;

    float t    = (float)(adc - dark_adc) / (float)(bright_adc - dark_adc);
    float duty = dark_duty + t * (bright_duty - dark_duty);
    return (uint8_t)duty;
}

/* ========================= Moving Average ========================= */

#define MA_WINDOW 5

static float ma_buffer[MA_WINDOW];
static uint8_t ma_index = 0;
static bool ma_filled = false;

static float moving_average(float new_sample)
{
    ma_buffer[ma_index] = new_sample;
    ma_index = (ma_index + 1) % MA_WINDOW;
    if (ma_index == 0) ma_filled = true;

    uint8_t count = ma_filled ? MA_WINDOW : ma_index;
    if (count == 0) return new_sample;
    
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++) sum += ma_buffer[i];
    return sum / (float)count;
}

/* ========================= Brake State Machine ========================= */

#define BRAKE_THRESHOLD_G  (-0.20f)
#define BRAKE_RELEASE_G    (-0.05f)
#define BRAKE_HOLD_MS      30
#define RAMP_DOWN_MS       500

#define BRAKE_DUTY         255  // Full bright on brake

typedef enum {
    STATE_BASELINE = 0,
    STATE_BRAKE,
    STATE_RAMP_DOWN
} brake_state_t;

static brake_state_t brake_state = STATE_BASELINE;
static uint16_t below_thresh_ms = 0;
static uint16_t ramp_elapsed_ms = 0;

static uint8_t brake_update(float ax_filtered, uint16_t dt_ms, uint8_t baseline_duty)
{
    switch (brake_state) {
    case STATE_BASELINE:
        if (ax_filtered < BRAKE_THRESHOLD_G) {
            below_thresh_ms += dt_ms;
            if (below_thresh_ms >= BRAKE_HOLD_MS) {
                brake_state = STATE_BRAKE;
                below_thresh_ms = 0;
                uart_println("\n>>> STATE: BASELINE -> BRAKE");
            }
        } else {
            below_thresh_ms = 0;
        }
        return baseline_duty;

    case STATE_BRAKE:
        if (ax_filtered > BRAKE_RELEASE_G) {
            brake_state = STATE_RAMP_DOWN;
            ramp_elapsed_ms = 0;
            uart_println(">>> STATE: BRAKE -> RAMP_DOWN");
        }
        return BRAKE_DUTY;

    case STATE_RAMP_DOWN:
        ramp_elapsed_ms += dt_ms;
        if (ramp_elapsed_ms >= RAMP_DOWN_MS) {
            brake_state = STATE_BASELINE;
            uart_println(">>> STATE: RAMP_DOWN -> BASELINE\n");
            return baseline_duty;
        }
        // Linear fade from BRAKE_DUTY to baseline_duty
        float t = (float)ramp_elapsed_ms / (float)RAMP_DOWN_MS;
        if (t > 1.0f) t = 1.0f;
        float duty_f = BRAKE_DUTY - t * (BRAKE_DUTY - baseline_duty);
        return (uint8_t)duty_f;
    }
    
    return baseline_duty;
}

/* ========================= MAIN ========================= */

int main(void)
{
    cli();
    
    uart_init(9600);
    uart_println("=== STEP 7: Full System + Ambient Light ===");
    
    timebase_init();
    pwm_init();
    ambient_init();
    
    uart_println("Initializing I2C...");
    TWI0_init();
    _delay_ms(100);
    
    uart_println("Initializing LSM6DSO...");
    if (!LSM6DS0_init()) {
        uart_println("FATAL: Sensor init failed!");
        while(1) {
            pwm_set_duty(255);
            _delay_ms(100);
            pwm_set_duty(0);
            _delay_ms(100);
        }
    }
    uart_println("Sensor initialized OK");
    
    sei();
    
    uart_println("\n*** Full brake light with ambient sensor ***");
    uart_println("Cover LDR (photoresistor) to make baseline dimmer");
    uart_println("Expose LDR to bright light to make baseline brighter\n");
    
    uint32_t last_read = 0;
    uint32_t last_ambient = 0;
    
    uint8_t baseline_duty = 10;  // Will be updated by ambient sensor
    
    // Get initial ambient reading
    baseline_duty = ambient_compute_duty();
    uart_print("Initial baseline: ");
    uart_print_int(baseline_duty);
    uart_print(" (ADC=");
    uart_print_int(read_adc0());
    uart_println(")");
    
    while (1) {
        uint32_t now = millis();
        
        // Read accelerometer every 50ms (20 Hz)
        uint16_t dt_ms = (uint16_t)(now - last_read);
        if (dt_ms >= 50) {
            last_read = now;
            
            int16_t ax_mg, ay_mg, az_mg;
            
            if (!LSM6DS0_read_accel_mg(&ax_mg, &ay_mg, &az_mg)) {
                uart_println("Read error!");
                continue;
            }
            
            float ax_g = ax_mg / 1000.0f;
            float ax_filtered = moving_average(ax_g);
            
            uint8_t duty = brake_update(ax_filtered, dt_ms, baseline_duty);
            pwm_set_duty(duty);
            
            // Status every second
            static uint8_t status_div = 0;
            if (++status_div >= 20) {
                status_div = 0;
                
                const char* state_names[] = {"BASELINE", "BRAKE", "RAMP_DOWN"};
                
                uart_print("raw=");
                uart_print_float(ax_g);
                uart_print("g, filt=");
                uart_print_float(ax_filtered);
                uart_print("g, PWM=");
                uart_print_int(duty);
                uart_print(", base=");
                uart_print_int(baseline_duty);
                uart_print(", state=");
                uart_println(state_names[brake_state]);
            }
        }
        
        // Update ambient light every 200ms
        if (now - last_ambient >= 200) {
            last_ambient = now;
            uint8_t new_baseline = ambient_compute_duty();
            
            // Only update if significantly different to avoid jitter
            if (new_baseline > baseline_duty + 2 || new_baseline < baseline_duty - 2) {
                baseline_duty = new_baseline;
                uart_print("Ambient changed: baseline=");
                uart_print_int(baseline_duty);
                uart_print(" (ADC=");
                uart_print_int(read_adc0());
                uart_println(")");
            }
        }
    }
}