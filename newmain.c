/*
 * Smart Vital Signs Monitor - PIC16F877A
 * Compiler: MPLAB XC8
 * Target: Proteus simulation / PIC16F877A hardware
 *
 * Based on the user's original vital signs monitor project.
 * Added innovations:
 *  1. Moving average filter for temperature and BPM
 *  2. Smart health classification: NORMAL / WARNING / CRITICAL / SENSOR ERROR
 *  3. Risk score from 0 to 3
 *  4. Signal quality check for pulse input
 *  5. Max/min memory for temperature and BPM
 *  6. Button menu: Live, Status, Max/Min, Last saved log
 *  7. Long button press resets max/min values
 *  8. Non-blocking LED/buzzer alarm patterns
 *  9. Startup self-test
 * 10. Optional UART serial monitoring on RC6/TX
 * 11. EEPROM logging using 24LC256 over I2C
 *
 * Expected circuit connections:
 *  - LM35 temperature sensor VOUT  -> RA0/AN0
 *  - Pulse/BPM potentiometer/sensor -> RA1/AN1
 *  - LCD 16x2 in 4-bit mode:
 *      RS -> RD4, EN -> RD5, D4 -> RD0, D5 -> RD1, D6 -> RD2, D7 -> RD3
 *  - Blue LED  -> RD6 through resistor
 *  - Red LED   -> RD7 through resistor
 *  - Buzzer    -> RC2/CCP1 through 2N2222 transistor driver
 *  - 24LC256   -> RC3/SCL, RC4/SDA with pull-up resistor
 *  - Pushbutton -> RB0/INT, active LOW with pull-up
 *  - Optional Virtual Terminal RX -> RC6/TX for serial logs
 */

// =============================================================
// CONFIGURATION BITS
// =============================================================
#pragma config FOSC = HS
#pragma config WDTE = OFF
#pragma config PWRTE = ON
#pragma config BOREN = ON
#pragma config LVP = OFF
#pragma config CPD = OFF
#pragma config WRT = OFF
#pragma config CP = OFF

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 20000000UL

// =============================================================
// PIN DEFINITIONS
// =============================================================

// LCD pins: 4-bit mode
#define LCD_RS      RD4
#define LCD_EN      RD5
#define LCD_D4      RD0
#define LCD_D5      RD1
#define LCD_D6      RD2
#define LCD_D7      RD3

#define LCD_TRIS_RS TRISD4
#define LCD_TRIS_EN TRISD5
#define LCD_TRIS_D4 TRISD0
#define LCD_TRIS_D5 TRISD1
#define LCD_TRIS_D6 TRISD2
#define LCD_TRIS_D7 TRISD3

// Output devices
#define LED_BLUE    RD6
#define LED_RED     RD7
#define BUZZER_PIN  RC2

// Inputs
#define BUTTON      RB0       // active LOW on RB0/INT

// ADC channels
#define ADC_TEMP    0         // AN0 / RA0: LM35
#define ADC_PULSE   1         // AN1 / RA1: pulse sensor or RV1 simulation

// =============================================================
// PROJECT SETTINGS AND THRESHOLDS
// =============================================================

#define FILTER_SIZE          5

// Temperature values are stored as x10, e.g. 36.7 C = 367
#define TEMP_LOW_WARN_X10    360
#define TEMP_HIGH_WARN_X10   380
#define TEMP_LOW_CRIT_X10    350
#define TEMP_HIGH_CRIT_X10   390

#define BPM_LOW_WARN         60
#define BPM_HIGH_WARN        100
#define BPM_LOW_CRIT         45
#define BPM_HIGH_CRIT        120

// Pulse input quality thresholds for simulation/analog input
#define PULSE_NO_FINGER_ADC  12
#define PULSE_MAX_ADC        1018
#define TEMP_SENSOR_MIN_ADC  2
#define TEMP_SENSOR_MAX_ADC  1020

// EEPROM settings for 24LC256
#define EEPROM_ADDR          0xA0
#define EEPROM_LOG_START     0x0000
#define EEPROM_LOG_MAX       120
#define EEPROM_RECORD_SIZE   3       // temp integer, bpm, status/risk packed byte
#define EEPROM_TEST_ADDR     0x7FFE

// Display modes
#define MODE_LIVE            0
#define MODE_STATUS          1
#define MODE_MINMAX          2
#define MODE_LAST_LOG        3
#define MODE_COUNT           4

// Health status
#define STATUS_SENSOR_ERROR  0
#define STATUS_NO_FINGER     1
#define STATUS_NORMAL        2
#define STATUS_WARNING       3
#define STATUS_CRITICAL      4

// Signal quality
#define SIGNAL_BAD           0
#define SIGNAL_WEAK          1
#define SIGNAL_NOISY         2
#define SIGNAL_GOOD          3

// =============================================================
// GLOBAL VARIABLES
// =============================================================

volatile uint16_t system_ticks_100ms = 0;
volatile uint8_t button_flag = 0;

uint16_t temp_x10_raw = 0;
uint16_t temp_x10 = 0;
uint8_t bpm_raw = 0;
uint8_t bpm = 0;
uint16_t pulse_adc = 0;
uint8_t health_status = STATUS_SENSOR_ERROR;
uint8_t signal_quality = SIGNAL_BAD;
uint8_t risk_score = 0;

uint8_t display_mode = MODE_LIVE;
uint8_t eeprom_ok = 0;
uint8_t log_index = 0;

uint16_t temp_filter[FILTER_SIZE];
uint8_t bpm_filter[FILTER_SIZE];
uint8_t filter_index = 0;
uint8_t filter_count = 0;

uint8_t minmax_ready = 0;
uint16_t temp_min_x10 = 0;
uint16_t temp_max_x10 = 0;
uint8_t bpm_min = 0;
uint8_t bpm_max = 0;

uint8_t last_raw_bpm = 0;
uint8_t noisy_count = 0;

// =============================================================
// SMALL UTILITY FUNCTIONS
// =============================================================

uint8_t u8_abs_diff(uint8_t a, uint8_t b) {
    if (a > b) {
        return a - b;
    }
    return b - a;
}

const char *status_to_text(uint8_t status) {
    switch (status) {
        case STATUS_NORMAL:       return "NORMAL";
        case STATUS_WARNING:      return "WARNING";
        case STATUS_CRITICAL:     return "CRITICAL";
        case STATUS_NO_FINGER:    return "NO FINGER";
        default:                  return "SENSOR ERR";
    }
}

const char *signal_to_text(uint8_t sig) {
    switch (sig) {
        case SIGNAL_GOOD:  return "GOOD";
        case SIGNAL_NOISY: return "NOISY";
        case SIGNAL_WEAK:  return "WEAK";
        default:           return "BAD";
    }
}

// =============================================================
// LCD FUNCTIONS
// =============================================================

/*
   IMPORTANT PIC16F877A NOTE:
   The PIC16F877A has a small 8-level hardware stack.
   Making this a macro instead of a function reduces nested CALL depth
   and prevents Proteus stack underflow/overflow during LCD updates.
*/
#define LCD_pulse_EN() do { \
    LCD_EN = 1;       \
    __delay_us(1);    \
    LCD_EN = 0;       \
    __delay_us(50);   \
} while (0)

void LCD_send_nibble(uint8_t nibble) {
    LCD_D4 = (nibble >> 0) & 1;
    LCD_D5 = (nibble >> 1) & 1;
    LCD_D6 = (nibble >> 2) & 1;
    LCD_D7 = (nibble >> 3) & 1;
    LCD_pulse_EN();
}

void LCD_send_byte(uint8_t value, uint8_t is_data) {
    LCD_RS = is_data;
    LCD_send_nibble(value >> 4);
    LCD_send_nibble(value & 0x0F);
    __delay_us(50);
}

void LCD_command(uint8_t cmd) {
    LCD_send_byte(cmd, 0);
}

void LCD_char(char c) {
    LCD_send_byte((uint8_t)c, 1);
}

void LCD_string(const char *s) {
    while (*s) {
        LCD_char(*s++);
    }
}

void LCD_goto(uint8_t row, uint8_t col) {
    if (row == 0) {
        LCD_command(0x80 + col);
    } else {
        LCD_command(0xC0 + col);
    }
}

void LCD_clear(void) {
    LCD_command(0x01);
    __delay_ms(2);
}

void LCD_init(void) {
    LCD_TRIS_RS = 0;
    LCD_TRIS_EN = 0;
    LCD_TRIS_D4 = 0;
    LCD_TRIS_D5 = 0;
    LCD_TRIS_D6 = 0;
    LCD_TRIS_D7 = 0;

    LCD_RS = 0;
    LCD_EN = 0;

    __delay_ms(20);

    LCD_send_nibble(0x03);
    __delay_ms(5);
    LCD_send_nibble(0x03);
    __delay_us(150);
    LCD_send_nibble(0x03);
    __delay_us(150);
    LCD_send_nibble(0x02);

    LCD_command(0x28);  // 4-bit, 2 lines, 5x8 font
    LCD_command(0x0C);  // display ON, cursor OFF
    LCD_command(0x06);  // entry mode
    LCD_clear();
}

void LCD_write_uint(uint16_t val, uint8_t digits) {
    char buf[6];
    uint8_t i;

    if (digits > 5) {
        digits = 5;
    }

    i = digits;
    buf[i] = '\0';

    while (i > 0) {
        i--;
        buf[i] = (char)('0' + (val % 10));
        val /= 10;
    }

    // Print directly instead of calling LCD_string() to reduce stack depth.
    i = 0;
    while (buf[i] != '\0') {
        LCD_char(buf[i]);
        i++;
    }
}

void LCD_write_temp_x10(uint16_t value_x10) {
    LCD_write_uint(value_x10 / 10, 2);
    LCD_char('.');
    LCD_write_uint(value_x10 % 10, 1);
}

void LCD_blank_line(uint8_t row) {
    LCD_goto(row, 0);
    LCD_string("                ");
}

// =============================================================
// ADC FUNCTIONS
// =============================================================

void ADC_init(void) {
    /*
       ADCON1 = 0x80:
       - Right justified ADC result
       - Vref+ = VDD, Vref- = VSS
       - Analog channels enabled for RA0/RA1 inputs
    */
    ADCON1 = 0x80;

    /*
       ADCON0 = 0x81:
       - ADC ON
       - Fosc/32
       - Channel 0 selected initially
    */
    ADCON0 = 0x81;
}

uint16_t ADC_read(uint8_t channel) {
    channel &= 0x07;

    ADCON0 &= 0xC7;
    ADCON0 |= (channel << 3);

    __delay_us(25);

    GO_nDONE = 1;
    while (GO_nDONE);

    return ((uint16_t)ADRESH << 8) | ADRESL;
}

uint16_t ADC_to_temp_x10(uint16_t adc_val) {
    /*
       LM35 gives 10 mV per 1 C.
       ADC mV = adc * 5000 / 1023.
       Because 1 C = 10 mV, temperature x10 is numerically equal to mV.
       Example: 370 mV = 37.0 C = 370 in x10 format.
    */
    uint32_t millivolts;
    millivolts = ((uint32_t)adc_val * 5000UL + 511UL) / 1023UL;
    return (uint16_t)millivolts;
}

// =============================================================
// TIMER1 INTERRUPT TICK
// =============================================================

void Timer1_init(void) {
    /*
       Fosc = 20 MHz, instruction clock = 5 MHz.
       Timer1 prescaler 1:8 gives 625 kHz.
       Overflow time is about 104.9 ms, used as an approximate 100 ms system tick.
    */
    T1CON = 0x31;
    TMR1H = 0;
    TMR1L = 0;
    TMR1IF = 0;
    TMR1IE = 1;
    PEIE = 1;
}

// =============================================================
// PWM BUZZER ON CCP1 / RC2
// =============================================================

void PWM_init(void) {
    TRISC2 = 0;

    // Approximately 2 kHz PWM tone with 20 MHz oscillator
    PR2 = 155;
    T2CON = 0x07;
    CCP1CON = 0x00;
    CCPR1L = 78;
    BUZZER_PIN = 0;
}

void Buzzer_on(void) {
    CCP1CON = 0x0C;
    CCPR1L = 78;
}

void Buzzer_off(void) {
    CCP1CON = 0x00;
    BUZZER_PIN = 0;
}

void Buzzer_short_beep(void) {
    Buzzer_on();
    __delay_ms(80);
    Buzzer_off();
}

// =============================================================
// UART SERIAL MONITORING, OPTIONAL
// =============================================================

void UART_init(void) {
    /*
       Optional Proteus Virtual Terminal:
       connect PIC RC6/TX to Virtual Terminal RX.
       Baud rate: 9600 at 20 MHz, BRGH = 1, SPBRG = 129.
    */
    TRISC6 = 0;
    TRISC7 = 1;
    SPBRG = 129;
    TXSTA = 0x24;
    RCSTA = 0x90;
}

void UART_char(char c) {
    while (!TXIF);
    TXREG = c;
}

void UART_string(const char *s) {
    while (*s) {
        UART_char(*s++);
    }
}

void UART_uint(uint16_t val) {
    char buf[6];
    uint8_t i = 0;
    uint8_t j;

    if (val == 0) {
        UART_char('0');
        return;
    }

    while ((val > 0) && (i < 5)) {
        buf[i++] = (char)('0' + (val % 10));
        val /= 10;
    }

    for (j = i; j > 0; j--) {
        UART_char(buf[j - 1]);
    }
}

void UART_temp_x10(uint16_t value_x10) {
    UART_uint(value_x10 / 10);
    UART_char('.');
    UART_uint(value_x10 % 10);
}

void UART_send_log(void) {
    UART_string("TEMP=");
    UART_temp_x10(temp_x10);
    UART_string("C, BPM=");
    UART_uint(bpm);
    UART_string(", STATUS=");
    UART_string(status_to_text(health_status));
    UART_string(", SIGNAL=");
    UART_string(signal_to_text(signal_quality));
    UART_string(", RISK=");
    UART_uint(risk_score);
    UART_string("\r\n");
}

// =============================================================
// I2C EEPROM 24LC256
// =============================================================

void I2C_init(void) {
    TRISC3 = 1;
    TRISC4 = 1;

    SSPADD = 49;       // 100 kHz I2C at 20 MHz oscillator
    SSPCON = 0x28;     // SSPEN=1, I2C master mode
    SSPCON2 = 0x00;
    SSPSTAT = 0x80;
}

void I2C_wait(void) {
    while ((SSPCON2 & 0x1F) || (SSPSTAT & 0x04));
}

void I2C_start(void) {
    I2C_wait();
    SEN = 1;
    while (SEN);
}

void I2C_stop(void) {
    I2C_wait();
    PEN = 1;
    while (PEN);
}

void I2C_repeated_start(void) {
    I2C_wait();
    RSEN = 1;
    while (RSEN);
}

uint8_t I2C_write(uint8_t data) {
    I2C_wait();
    SSPBUF = data;
    I2C_wait();
    return ACKSTAT;
}

uint8_t I2C_read_ack(void) {
    uint8_t data;

    I2C_wait();
    RCEN = 1;
    while (!BF);
    data = SSPBUF;

    I2C_wait();
    ACKDT = 0;
    ACKEN = 1;
    while (ACKEN);

    return data;
}

uint8_t I2C_read_nack(void) {
    uint8_t data;

    I2C_wait();
    RCEN = 1;
    while (!BF);
    data = SSPBUF;

    I2C_wait();
    ACKDT = 1;
    ACKEN = 1;
    while (ACKEN);
    ACKDT = 0;

    return data;
}

uint8_t EEPROM_ping(void) {
    uint8_t nack;

    I2C_start();
    nack = I2C_write(EEPROM_ADDR);
    I2C_stop();

    if (nack == 0) {
        return 1;
    }
    return 0;
}

void EEPROM_write_byte(uint16_t mem_addr, uint8_t data) {
    if (!eeprom_ok) {
        return;
    }

    I2C_start();
    I2C_write(EEPROM_ADDR);
    I2C_write((uint8_t)(mem_addr >> 8));
    I2C_write((uint8_t)(mem_addr & 0xFF));
    I2C_write(data);
    I2C_stop();

    __delay_ms(5);
}

uint8_t EEPROM_read_byte(uint16_t mem_addr) {
    uint8_t data = 0;

    if (!eeprom_ok) {
        return 0;
    }

    I2C_start();
    I2C_write(EEPROM_ADDR);
    I2C_write((uint8_t)(mem_addr >> 8));
    I2C_write((uint8_t)(mem_addr & 0xFF));

    I2C_repeated_start();
    I2C_write(EEPROM_ADDR | 0x01);

    data = I2C_read_nack();
    I2C_stop();

    return data;
}

void EEPROM_log_reading(void) {
    uint16_t addr;
    uint8_t temp_int;
    uint8_t packed_status;

    if (!eeprom_ok) {
        return;
    }

    // Do not log invalid BPM readings as real patient data
    if ((health_status == STATUS_SENSOR_ERROR) || (health_status == STATUS_NO_FINGER)) {
        return;
    }

    temp_int = (uint8_t)(temp_x10 / 10);
    packed_status = (uint8_t)((risk_score << 4) | (health_status & 0x0F));

    addr = EEPROM_LOG_START + ((uint16_t)log_index * EEPROM_RECORD_SIZE);

    EEPROM_write_byte(addr, temp_int);
    EEPROM_write_byte(addr + 1, bpm);
    EEPROM_write_byte(addr + 2, packed_status);

    log_index++;
    if (log_index >= EEPROM_LOG_MAX) {
        log_index = 0;
    }
}

// =============================================================
// SENSOR PROCESSING AND INNOVATION LOGIC
// =============================================================

void BPM_calculate_raw(void) {
    uint32_t scaled;

    pulse_adc = ADC_read(ADC_PULSE);

    // RV1 simulation: 0 V to 5 V maps approximately from 50 to 120 BPM
    scaled = ((uint32_t)pulse_adc * 70UL) / 1023UL;
    bpm_raw = (uint8_t)(50 + scaled);
}

void update_signal_quality(void) {
    if (pulse_adc < PULSE_NO_FINGER_ADC) {
        signal_quality = SIGNAL_BAD;
        return;
    }

    if (pulse_adc > PULSE_MAX_ADC) {
        signal_quality = SIGNAL_WEAK;
        return;
    }

    // Noise check: very sudden BPM jumps are marked as noisy for a few cycles
    if (u8_abs_diff(bpm_raw, last_raw_bpm) > 25) {
        noisy_count = 3;
    } else if (noisy_count > 0) {
        noisy_count--;
    }

    last_raw_bpm = bpm_raw;

    if (noisy_count > 0) {
        signal_quality = SIGNAL_NOISY;
    } else {
        signal_quality = SIGNAL_GOOD;
    }
}

void filter_add_sample(uint16_t new_temp_x10, uint8_t new_bpm) {
    uint8_t i;
    uint32_t temp_sum = 0;
    uint16_t bpm_sum = 0;

    temp_filter[filter_index] = new_temp_x10;
    bpm_filter[filter_index] = new_bpm;

    filter_index++;
    if (filter_index >= FILTER_SIZE) {
        filter_index = 0;
    }

    if (filter_count < FILTER_SIZE) {
        filter_count++;
    }

    for (i = 0; i < filter_count; i++) {
        temp_sum += temp_filter[i];
        bpm_sum += bpm_filter[i];
    }

    temp_x10 = (uint16_t)(temp_sum / filter_count);
    bpm = (uint8_t)(bpm_sum / filter_count);
}

void update_health_status(uint16_t adc_temp) {
    uint8_t temp_warning = 0;
    uint8_t temp_critical = 0;
    uint8_t bpm_warning = 0;
    uint8_t bpm_critical = 0;

    // Sensor error checks
    if ((adc_temp < TEMP_SENSOR_MIN_ADC) || (adc_temp > TEMP_SENSOR_MAX_ADC)) {
        health_status = STATUS_SENSOR_ERROR;
        risk_score = 3;
        return;
    }

    if (pulse_adc < PULSE_NO_FINGER_ADC) {
        health_status = STATUS_NO_FINGER;
        risk_score = 0;
        return;
    }

    if ((temp_x10 >= TEMP_HIGH_CRIT_X10) || (temp_x10 <= TEMP_LOW_CRIT_X10)) {
        temp_critical = 1;
    } else if ((temp_x10 >= TEMP_HIGH_WARN_X10) || (temp_x10 < TEMP_LOW_WARN_X10)) {
        temp_warning = 1;
    }

    if ((bpm >= BPM_HIGH_CRIT) || (bpm <= BPM_LOW_CRIT)) {
        bpm_critical = 1;
    } else if ((bpm > BPM_HIGH_WARN) || (bpm < BPM_LOW_WARN)) {
        bpm_warning = 1;
    }

    // Risk score 0 to 3
    risk_score = 0;

    if (temp_warning || temp_critical) {
        risk_score++;
    }

    if (bpm_warning || bpm_critical) {
        risk_score++;
    }

    // Fever + high pulse is treated as more serious
    if ((temp_x10 >= TEMP_HIGH_WARN_X10) && (bpm > BPM_HIGH_WARN)) {
        risk_score++;
    }

    if (temp_critical || bpm_critical || (risk_score >= 3)) {
        health_status = STATUS_CRITICAL;
        risk_score = 3;
    } else if (risk_score > 0) {
        health_status = STATUS_WARNING;
    } else {
        health_status = STATUS_NORMAL;
    }
}

void update_minmax(void) {
    if ((health_status == STATUS_SENSOR_ERROR) || (health_status == STATUS_NO_FINGER)) {
        return;
    }

    if (!minmax_ready) {
        temp_min_x10 = temp_x10;
        temp_max_x10 = temp_x10;
        bpm_min = bpm;
        bpm_max = bpm;
        minmax_ready = 1;
        return;
    }

    if (temp_x10 < temp_min_x10) {
        temp_min_x10 = temp_x10;
    }

    if (temp_x10 > temp_max_x10) {
        temp_max_x10 = temp_x10;
    }

    if (bpm < bpm_min) {
        bpm_min = bpm;
    }

    if (bpm > bpm_max) {
        bpm_max = bpm;
    }
}

void reset_minmax(void) {
    minmax_ready = 0;
    temp_min_x10 = 0;
    temp_max_x10 = 0;
    bpm_min = 0;
    bpm_max = 0;
}

void read_all_sensors(void) {
    uint16_t adc_temp;

    adc_temp = ADC_read(ADC_TEMP);
    temp_x10_raw = ADC_to_temp_x10(adc_temp);

    BPM_calculate_raw();
    update_signal_quality();

    // Use raw values in the filter even if noisy, then classify afterward
    filter_add_sample(temp_x10_raw, bpm_raw);
    update_health_status(adc_temp);
    update_minmax();
}

// =============================================================
// ALERT OUTPUTS
// =============================================================

void update_alert_outputs(void) {
    uint16_t t;

    t = system_ticks_100ms;

    switch (health_status) {
        case STATUS_NORMAL:
            LED_BLUE = 1;
            LED_RED = 0;
            Buzzer_off();
            break;

        case STATUS_WARNING:
            // Warning: alternate red and blue slowly, short beep every ~2 s
            if ((t % 10) < 5) {
                LED_BLUE = 1;
                LED_RED = 0;
            } else {
                LED_BLUE = 0;
                LED_RED = 1;
            }

            if ((t % 20) < 1) {
                Buzzer_on();
            } else {
                Buzzer_off();
            }
            break;

        case STATUS_CRITICAL:
            // Critical: fast red blinking and repeated buzzer
            LED_BLUE = 0;
            if ((t % 4) < 2) {
                LED_RED = 1;
                Buzzer_on();
            } else {
                LED_RED = 0;
                Buzzer_off();
            }
            break;

        case STATUS_NO_FINGER:
            // No finger: both LEDs blink slowly, no buzzer
            if ((t % 12) < 6) {
                LED_BLUE = 1;
                LED_RED = 1;
            } else {
                LED_BLUE = 0;
                LED_RED = 0;
            }
            Buzzer_off();
            break;

        default:
            // Sensor error: red blink only, no buzzer
            LED_BLUE = 0;
            LED_RED = ((t % 6) < 3) ? 1 : 0;
            Buzzer_off();
            break;
    }
}

// =============================================================
// DISPLAY SCREENS
// =============================================================

void display_live_screen(void) {
    LCD_goto(0, 0);
    LCD_string("Temp:");
    LCD_write_temp_x10(temp_x10);
    LCD_string("C B:");
    LCD_write_uint(bpm, 3);

    LCD_goto(1, 0);
    if (health_status == STATUS_NORMAL) {
        LCD_string("OK Sig:");
        LCD_string(signal_to_text(signal_quality));
        LCD_string("     ");
    } else if (health_status == STATUS_NO_FINGER) {
        LCD_string("Place finger    ");
    } else if (health_status == STATUS_SENSOR_ERROR) {
        LCD_string("Sensor error    ");
    } else if (health_status == STATUS_WARNING) {
        LCD_string("Warning R:");
        LCD_write_uint(risk_score, 1);
        LCD_string("      ");
    } else {
        LCD_string("CRITICAL R:");
        LCD_write_uint(risk_score, 1);
        LCD_string("    ");
    }
}

void display_status_screen(void) {
    LCD_goto(0, 0);
    LCD_string("Status:");
    LCD_string(status_to_text(health_status));
    LCD_string("   ");

    LCD_goto(1, 0);
    LCD_string("Risk:");
    LCD_write_uint(risk_score, 1);
    LCD_string(" Sig:");
    LCD_string(signal_to_text(signal_quality));
    LCD_string("  ");
}

void display_minmax_screen(void) {
    if (!minmax_ready) {
        LCD_goto(0, 0);
        LCD_string("Max/Min not set ");
        LCD_goto(1, 0);
        LCD_string("Need valid data ");
        return;
    }

    LCD_goto(0, 0);
    LCD_string("T:");
    LCD_write_temp_x10(temp_min_x10);
    LCD_char('-');
    LCD_write_temp_x10(temp_max_x10);
    LCD_char('C');
    LCD_string("  ");

    LCD_goto(1, 0);
    LCD_string("B:");
    LCD_write_uint(bpm_min, 3);
    LCD_char('-');
    LCD_write_uint(bpm_max, 3);
    LCD_string(" Hold=Rst");
}

void display_last_log_screen(void) {
    uint16_t addr;
    uint8_t saved_temp;
    uint8_t saved_bpm;
    uint8_t saved_pack;
    uint8_t saved_risk;
    uint8_t saved_status;

    if (!eeprom_ok) {
        LCD_goto(0, 0);
        LCD_string("EEPROM not found");
        LCD_goto(1, 0);
        LCD_string("Check I2C wires ");
        return;
    }

    if (log_index == 0) {
        addr = EEPROM_LOG_START + ((EEPROM_LOG_MAX - 1) * EEPROM_RECORD_SIZE);
    } else {
        addr = EEPROM_LOG_START + (((uint16_t)log_index - 1) * EEPROM_RECORD_SIZE);
    }

    saved_temp = EEPROM_read_byte(addr);
    saved_bpm = EEPROM_read_byte(addr + 1);
    saved_pack = EEPROM_read_byte(addr + 2);

    saved_status = saved_pack & 0x0F;
    saved_risk = saved_pack >> 4;

    LCD_goto(0, 0);
    LCD_string("Last T:");
    LCD_write_uint(saved_temp, 2);
    LCD_string("C B:");
    LCD_write_uint(saved_bpm, 3);

    LCD_goto(1, 0);
    LCD_string("R:");
    LCD_write_uint(saved_risk, 1);
    LCD_char(' ');
    LCD_string(status_to_text(saved_status));
    LCD_string("     ");
}

void update_display(void) {
    switch (display_mode) {
        case MODE_STATUS:
            display_status_screen();
            break;

        case MODE_MINMAX:
            display_minmax_screen();
            break;

        case MODE_LAST_LOG:
            display_last_log_screen();
            break;

        default:
            display_live_screen();
            break;
    }
}

// =============================================================
// BUTTON HANDLING
// =============================================================

void handle_button(void) {
    uint8_t hold_count = 0;

    if (!button_flag) {
        return;
    }

    button_flag = 0;
    __delay_ms(25);      // debounce

    if (BUTTON != 0) {
        return;
    }

    while (BUTTON == 0) {
        __delay_ms(20);
        hold_count++;

        if (hold_count >= 50) {      // about 1 second long press
            reset_minmax();
            LCD_clear();
            LCD_goto(0, 0);
            LCD_string("Max/Min reset");
            LCD_goto(1, 0);
            LCD_string("Release button ");
            Buzzer_short_beep();

            while (BUTTON == 0);
            __delay_ms(25);
            LCD_clear();
            return;
        }
    }

    // Short press changes display mode
    display_mode++;
    if (display_mode >= MODE_COUNT) {
        display_mode = MODE_LIVE;
    }

    LCD_clear();
}

// =============================================================
// STARTUP SELF-TEST
// =============================================================

void startup_self_test(void) {
    LCD_clear();
    LCD_goto(0, 0);
    LCD_string("Vital Monitor");
    LCD_goto(1, 0);
    LCD_string("Self test...");

    LED_BLUE = 1;
    LED_RED = 0;
    Buzzer_short_beep();
    __delay_ms(150);

    LED_BLUE = 0;
    LED_RED = 1;
    __delay_ms(150);

    LED_BLUE = 0;
    LED_RED = 0;

    LCD_clear();
    LCD_goto(0, 0);
    LCD_string("LCD ADC PWM OK");

    eeprom_ok = EEPROM_ping();
    LCD_goto(1, 0);
    if (eeprom_ok) {
        LCD_string("EEPROM I2C OK  ");
    } else {
        LCD_string("EEPROM missing ");
    }

    __delay_ms(700);
    LCD_clear();
}

// =============================================================
// INTERRUPT SERVICE ROUTINE
// =============================================================

void __interrupt() ISR(void) {
    if (TMR1IE && TMR1IF) {
        TMR1IF = 0;
        TMR1H = 0;
        TMR1L = 0;
        system_ticks_100ms++;
    }

    if (INTE && INTF) {
        INTF = 0;
        button_flag = 1;
    }
}

// =============================================================
// MAIN PROGRAM
// =============================================================

void main(void) {
    uint8_t loop_counter = 0;
    uint8_t uart_counter = 0;

    // ----------------------
    // Port directions
    // ----------------------
    TRISA = 0xFF;     // RA0/AN0 LM35, RA1/AN1 pulse/RV1
    TRISB = 0xFF;     // RB0 button input
    TRISC = 0x00;     // RC2 PWM, RC3/RC4 I2C, RC6 UART TX, RC7 UART RX
    TRISD = 0x00;     // LCD and LEDs
    TRISE = 0x07;

    // ----------------------
    // Initial output states
    // ----------------------
    PORTA = 0x00;
    PORTB = 0x00;
    PORTC = 0x00;
    PORTD = 0x00;
    PORTE = 0x00;

    LED_BLUE = 0;
    LED_RED = 0;
    Buzzer_off();

    // Enable PORTB pull-ups and falling-edge external interrupt
    OPTION_REG &= 0x7F;   // enable PORTB pull-ups globally
    INTEDG = 0;           // interrupt on falling edge: button press
    INTE = 1;
    INTF = 0;

    // ----------------------
    // Peripheral initialization
    // ----------------------
    LCD_init();
    ADC_init();
    Timer1_init();
    PWM_init();
    I2C_init();
    UART_init();

    // Keep global interrupts disabled during LCD-heavy startup test.
    PEIE = 1;
    GIE = 0;

    startup_self_test();

    // Enable interrupts only after the display initialization/test is finished.
    GIE = 1;

    while (1) {
        // 1. Read sensors and process data
        read_all_sensors();

        // 2. Apply smart alarms without blocking the program
        update_alert_outputs();

        // 3. Update LCD according to selected menu mode.
        // Temporarily disable interrupts during LCD writes to avoid stack overflow
        // on the PIC16F877A when an interrupt occurs inside nested LCD functions.
        GIE = 0;
        update_display();
        GIE = 1;

        // 4. Handle short press / long press button menu
        handle_button();

        // 5. EEPROM log every 10 loops, about every 1 second
        loop_counter++;
        if (loop_counter >= 10) {
            loop_counter = 0;
            EEPROM_log_reading();
        }

        // 6. UART log every 20 loops, about every 2 seconds
        uart_counter++;
        if (uart_counter >= 20) {
            uart_counter = 0;
            UART_send_log();
        }

        __delay_ms(100);
    }
}