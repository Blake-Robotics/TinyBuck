/* tinyBuck firmware
 *
 * Firmware to drive the tinyBuck led patterns. Provides an
 * i2c interface and basic PWM output. Parameters are stored
 * in the EEPROM.
 *
 * Chip Pinout:
 *   
 *  PA0 - PWM0
 *  PA1 - PWM1
 *  PA2 - PWM2
 *
 * To compile & link:
 *  avr-gcc -g -Os -mmcu=attiny24 -c main.c
 *  avr-gcc -g -mmcu=attiny24 -o main.elf main.o
 *
 * To convert to binary file for upload to ATTINY:
 *  avr-objcopy -j .text -j .data -O ihex main.elf main.hex
 *  avrdude -P /dev/ttyACM0 -b 19200 -c avrisp -p attiny24 -U flash:w:main.hex
 *
 */


/*****************************************************************************
 * Includes
 *****************************************************************************/ 
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <string.h>

// I2C Slave:
#include "usiTwiSlave.h"

// Project includes
#include "TinyBuckDefines.h"
#include "eeprom.h"

/*****************************************************************************
 * Global Variables
 *****************************************************************************/
volatile uint8_t pwm_counter;
volatile uint8_t pwm_next;
volatile uint8_t pwm0, pwm1, pwm2;
volatile uint16_t extended_time;

effect_cfg effect_now, effect_buf;



/*****************************************************************************
 * Interrupt handlers
 *****************************************************************************/

// Software PWM Interrupt handler.
ISR(TIM0_COMPA_vect)
{
    // Update the PWM, then allow other interrupts
    // I2C, I'm looking at you!
    PORTA = (PORTA & 0xF8) | pwm_next;
    sei();	

    uint8_t temp = pwm_counter;  
    pwm_next = ((temp < pwm0) ? 0x1 : 0x0) | 
            ((temp < pwm1) ? 0x2 : 0x0) |
            ((temp < pwm2) ? 0x4 : 0x0);
    pwm_counter = temp + 1;
}

// Global timer handler:
ISR(TIM1_OVF_vect) {
    extended_time++;
}


void i2c_poll(void);
void effect(void);


/*****************************************************************************
 * Main Program
 *****************************************************************************/
int main (void)
{

    // Software PWM setup:
    DDRA = 0x7;
    PORTA = 0x0;
    pwm_counter = 0;
    pwm_next = 0x0;
    
    // Setup 8-bit timer to generate interrupts for software PWM:
    TCCR0A = (1<<WGM01); // Normal operation  
    TCCR0B = (1<<CS00);  // Set coutner prescaler to 1 (8MHz)
    //TCCR0B = (1<<CS01);  // Set coutner prescaler to 8 (1MHz)
    TIMSK0 |= 1<<OCIE0A; // Enable interrupt on compare
    OCR0A = 100;
    TCNT0 = 0;

    _delay_ms(0.01);

    // Setup 16-bit timer to keep track of internal time:
    TCCR1A = 0x0;       // Normal operation
    TCCR1B = (1<<CS10); // Set prescaler to 1.
    TIMSK1 |= (1<<TOIE1); // Interrupt on overflow
    TCNT1 = 0;

    sei();  // enable interrupts

    // Setup I2C:
    //usiTwiSlaveInit(read_eeprom_byte(EEPROM_I2CADDR));
    usiTwiSlaveInit(0x14);

    // Load effect configuration from EEPROM:
    read_eeprom_array(EEPROM_EFFECT, effect_buf, TB_EFFECT_SIZE);
    memcpy(effect_now, effect_buf, TB_EFFECT_SIZE);
    //pwm0 = 0x3;

    // Start the program:
    
    for (;;)
    {
        i2c_poll();
        effect();
    }
};

/*****************************************************************************
 * Handle I2C Commands
 *****************************************************************************/
void i2c_poll(void)
{
    uint8_t addr; // address byte
    uint8_t data; // data byte

    if (usiTwiAmountDataInReceiveBuffer() > 1)
    {
        addr = usiTwiReceiveByte();
        data = usiTwiReceiveByte();

        // Special opcodes.
        if (addr < TB_SYNC_BEGIN)
        {
            switch (addr)
            {
                case TB_RESET:
                      break;

                case TB_UPDATE:
                      memcpy(effect_now, effect_buf, TB_EFFECT_SIZE);
                      break;

                case TB_SAVE:
                      update_eeprom_array(EEPROM_EFFECT,effect_buf,TB_EFFECT_SIZE);
                      break;

                case TB_SETI2C:
                      if ((data > 3) && (data < 77)) write_eeprom_byte(EEPROM_I2CADDR, data);
                      break;



            }

        }
        // synchronser configuration.
        else if (addr < TB_EFFECT_CFG)
        {
            

        }
        // effect configuration
        else if (addr < TB_EFFECT_CFG+TB_EFFECT_SIZE)
        {
            effect_buf[addr-TB_EFFECT_CFG] = data;
        }

    }

};

/*****************************************************************************
 * Handle effect generation
 *****************************************************************************/
void effect(void)
{
    // Static PWM effect:
    if (effect_now[0] == 0)
    {
        pwm0 = effect_now[1];
        pwm1 = effect_now[2];
        pwm2 = effect_now[3];
    }

    // Ramping up effect
    if (effect_now[0] == 1)
    {
        uint8_t temp = (uint16_t) extended_time/2;
        if (temp < 128)
        {
            pwm0 = temp*2;
        }
        else
        {
            pwm0 = 255 - temp*2;
        }
        pwm1 = 255-pwm0;
        pwm2 = 0;
    }

};
