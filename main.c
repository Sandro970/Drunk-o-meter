#define F_CPU 8000000UL
#define KEY1 0
#define KEY2 1
#define KEY3 2
#define KEY4 3
#define LCD_PIN 4
#define BUZZER_PIN 7
#define OCR0_VALUE 243 // calculated value for timer
#define OCR1B_VALUE 2
#define AVERAGE_ARRAY_SIZE 10
#define ADC_STRING_SIZE 128
#define DELAY_TIME 200
#define BUZZER_DELAY 100
#define MINIMUM_REFERENCE_VALUE 0
#define MAXIMUM_REFERENCE_VALUE 1000
#define REFERENCE_VALUE_CHANGE 25
#define INITIAL_OSCILLATIONS_VALUE 10
#define NOT_SET 0
#define FIRST_MODE 1
#define SECOND_MODE 2

#include <stdio.h>
#include <stdlib.h>
#include "lcd.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

uint8_t isCalibrated = 0;
uint8_t delayTo32 = 0;
uint8_t mode = 0;
uint8_t averageArrayIndex = 0;
uint8_t currentTime = 0;
uint16_t averageArray[AVERAGE_ARRAY_SIZE]; //an array used to store values of which we later need the average
uint16_t maxValue = 0;
uint16_t lastCalibratedValue = 0;
uint16_t calibratedValue = 0;

uint16_t arrayAverage(void) {
	uint16_t sum = 0;
	for (uint8_t i = 0; i < AVERAGE_ARRAY_SIZE; ++i) {
		sum += averageArray[i];
	}
	return sum / AVERAGE_ARRAY_SIZE;
}

ISR(TIMER0_COMP_vect) {
	char adcString[ADC_STRING_SIZE];
	uint8_t absoluteDifference;

	if (delayTo32 >= 32) { 
		delayTo32 = 0;
		absoluteDifference = ADC > lastCalibratedValue ? ADC - lastCalibratedValue : lastCalibratedValue - ADC;
		if (absoluteDifference > INITIAL_OSCILLATIONS_VALUE) { // waits out initial oscillations
			lcd_clrscr();
			snprintf(adcString, ADC_STRING_SIZE, "Calibrating..\nTime: %ds", currentTime);
			lastCalibratedValue = ADC;
			lcd_puts(adcString);
		} else {
			isCalibrated = 1;
		}
		++currentTime;
	}
	++delayTo32;
}

void writeLCD(uint16_t currentValue) {
	uint16_t averageArrayValue;
	uint16_t alcoholReferenceValue;
	char adcString[ADC_STRING_SIZE];
	lcd_clrscr();

	switch (mode) {
		case 0: // normal mode
		snprintf(adcString, ADC_STRING_SIZE, "%3d | MAX:%4d", currentValue, maxValue);
		break;
		case 1: // percentage mode
		snprintf(adcString, ADC_STRING_SIZE, "%3d | MAX:%4d\n%3d%% of %3d", currentValue, maxValue, (int)(currentValue * 100.0 / calibratedValue), calibratedValue);
		break;
		case 2: // drunk-o-meter mode
		alcoholReferenceValue = eeprom_read_word(0);
		averageArrayValue = arrayAverage();
		if (averageArrayValue >= alcoholReferenceValue) {
			snprintf(adcString, ADC_STRING_SIZE, "Go home, drunk!\nAVG: %3d>=%3d", averageArrayValue, alcoholReferenceValue);
			PORTC = 0x00; //buzzer ON
			_delay_ms(BUZZER_DELAY);
			PORTC = 0xff; //buzzer OFF
			_delay_ms(BUZZER_DELAY);
			} else {
			snprintf(adcString, ADC_STRING_SIZE, "Drive safely!\nAVG: %3d<%3d", averageArrayValue, alcoholReferenceValue);
		}
		break;
		default:
		snprintf(adcString, ADC_STRING_SIZE, "There has been an unknown error.");
		break;
	}

	lcd_puts(adcString);
}

void initializeCalibrationTimer() {
	TCCR0 = _BV(WGM01) | _BV(COM00) | _BV(CS02) | _BV(CS00); //CTC - toggle OC0 on compare match, prescaler 1024
	OCR0 = OCR0_VALUE;
	TIMSK = _BV(OCIE0);
	sei();
}

void pushValue(uint16_t val) {
	averageArrayIndex %= AVERAGE_ARRAY_SIZE;
	averageArray[averageArrayIndex++] = val;
}

int main(void) {
	PORTB |= _BV(KEY1) | _BV(KEY2) | _BV(KEY3) | _BV(KEY4);
	DDRB &= ~(_BV(KEY1) | _BV(KEY2) | _BV(KEY3) | _BV(KEY4));

	// initialize LCD
	DDRD = _BV(LCD_PIN);
	
	//initialize buzzer
	DDRC = _BV(BUZZER_PIN);
	PORTC = 0xff; //turn off buzzer

	TCCR1A = _BV(COM1B1) | _BV(WGM10);
	TCCR1B = _BV(WGM12) | _BV(CS11);
	OCR1B = OCR1B_VALUE;

	lcd_init(LCD_DISP_ON);

	ADMUX = _BV(REFS0);
	ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1);

	initializeCalibrationTimer();

	while (!isCalibrated) {
		ADCSRA |= _BV(ADSC);
		while (!(ADCSRA & _BV(ADIF))) {
			;
		}
		_delay_ms(DELAY_TIME);
	}
	cli(); // kill calibration timer
	while (1) { // finished calibrating
		ADCSRA |= _BV(ADSC);
		while (!(ADCSRA & _BV(ADIF))) {
			;
		}

		if (mode != SECOND_MODE) { // don't update measurement if using drunk-o-meter
			pushValue(ADC);
		}

		if (ADC > maxValue) {
			maxValue = ADC;
		}
		if (bit_is_clear(PINB, KEY1)) { // KEY1 is pressed
			if (mode == NOT_SET) {
				mode = FIRST_MODE;
				if (calibratedValue == NOT_SET) { // If not calibrated yet
					if (ADC != NOT_SET) {
						calibratedValue = ADC;
					} else {
						mode = NOT_SET;
					}
				}
			} else {
				mode = NOT_SET;
			}
		} else if (bit_is_clear(PINB, KEY2)) { // KEY2 is pressed
			if (mode == SECOND_MODE) {
				mode = NOT_SET;
			} else {
				mode = SECOND_MODE;
			}
		} else if (bit_is_clear(PINB, KEY3)) {
			uint16_t temporaryValue = eeprom_read_word(0);
			if (temporaryValue > MINIMUM_REFERENCE_VALUE) {
				eeprom_write_word(0, temporaryValue - REFERENCE_VALUE_CHANGE);
			}
		} else if (bit_is_clear(PINB, KEY4)) {
			uint16_t temporaryValue = eeprom_read_word(0);
			if (temporaryValue < MAXIMUM_REFERENCE_VALUE) {
				eeprom_write_word(0, temporaryValue + REFERENCE_VALUE_CHANGE);
			}
		}
		writeLCD(ADC);

		_delay_ms(DELAY_TIME); // debounce
	}
}