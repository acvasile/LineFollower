/* (C) 2018 Vasile Alexandru-Catalin
 * Line follower project for Microcontroller Programming
 */
#include <avr/io.h>
#include <util/delay.h>

/* Enable USART debugging */
//#define USART_DEBUG

#ifdef USART_DEBUG
#include "usart.h"
#endif

/* Maximum message length send over USART */
#define USART_BUF_LEN 256

/* Do not modify cpu frequency */
#define F_CPU 16000000

/* In case the motors are not performing equally */
#define LEFT_MAX_SPEED	220
#define RIGHT_MAX_SPEED 210


#define SENSOR_NUM	    6		/* Number of sensors */
#define SLEEP_BETWEEN	15		/* Sleep between loops (battery consumption) */

#define NOISE_THRESHOLD 50		/* Values bellow this are not valid */
#define ON_TRACK_VALUE	200		/* If value is above this then the sensor is on the line */


#define BASE_SPEED		175		/* Base speed of motors */
#define MAX_SPEED		240		/* Maximum speed of motors */

#define HEUR_OFFSET     1000    /* The heuristic value offset */
#define MIDDLE_ERROR    2500    /* The value representative for a straight line */
/* Unexpected behaviour happens when the integral performs too many
 * iterations leading to a decrease in performance.
 */
#define INTEGRAL_RESET_ITER 125


/*
 Constant table:
 ---------
 Slow but handles all corner cases
 kp			0.025
 BASE_SPEED	75
 MAX_SPEED	200
 ---------
 Fast			(low battery consumption )
 kp			0.00382
 BASE_SPEED	125
 MAX_SPEED	240
 ---------
 Even faster	(high battery consumption)
 kp			0.0515
 BASE_SPEED	175
 MAX_SPEED	240

 Difference between case 2 and case 3 is about 1.2s
 */

/* Following values are related to each other
 * but they are also influenced by BASE_SPEED and MAX_SPEED + SLEEP_BETWEEN
 */
float KP = 0.0515;
float KD = 0.00035;
float KI = 0.00000267;

/* Typedef boolean type */
typedef enum { false, true } bool;

uint8_t clamp(int value, uint8_t min, uint8_t max)
{
	if (value < min) { return min; }
	if (value > max) { return max; }
	return value;
}

void ADC_init()
{
	ADMUX = (1 << REFS0);
	/* Enable ADC and set PS at 128 */
	ADCSRA = (1 << ADEN) | (7 << ADPS0);
}

int ADC_get(uint8_t channel)
{
	ADMUX = (ADMUX & ~(0x1f << MUX0)) | channel;
	ADCSRA |= (1 << ADSC);

	while (ADCSRA & (1 << ADSC));

	return ADC;
}

void read_line_input(int arr[])
{
	int i = 0;

	for (i = 0; i < SENSOR_NUM; i++)
	{
		arr[i] = ADC_get(i);
	}
}

void engine_init()
{
    /* Compiler will simplify to 0xF3
     * Clear on button with FastPWM
     */
	TCCR0A = (1 << COM0A1) | (1 << COM0A0) | (1 << COM0B1) | (1 << COM0B0);
	TCCR0A |= (1 << WGM01) | (1 << WGM00);

	TCCR2A = (1 << COM2A1) | (1 << COM2A0) | (1 << COM2B1) | (1 << COM2B0);
	TCCR2A |= (1 << WGM21) | (1 << WGM20);

    /* PS at 8 */
	TCCR0B = (1 << CS01);
	TCCR2B = (1 << CS21);

	/* Clear values */
	OCR0A = 0;
	OCR0B = 0;
	
	OCR2A = 0;
	OCR2B = 0;
	
	/* Output pins for engines */
	DDRD |= (1 << PD6);
	DDRD |= (1 << PD7);
	PORTD &= ~(1 << PD6);
	PORTD &= ~(1 << PD7);

	DDRB |= (1 << PB3);
	DDRB |= (1 << PB4);
	PORTB &= ~(1 << PB3);
	PORTB &= ~(1 << PB4);
}

void left_motor_set(uint8_t strength)
{
	OCR0A = strength;
	OCR0B = 0;
}

void right_motor_set(uint8_t strength)
{
	OCR2A = 0;
	OCR2B = strength;
}

/* Adjusted function from QTR-8A reading implementation by Polulu
 * This returns an even average of inputs that is defined
 * by the following heuristic:
 * (sens0 * 0 + sens1 * 1000 + sens3 * 2000 + ...) / (sens0 + sens1 + ...)
 */
int read_qrt_line(int values[])
{
	static long last_value = 0;
	unsigned long average = 0;
	unsigned int sum = 0;
	unsigned int heur_offset = 0;
	bool on_track = false;
	int i = 0;

	read_line_input(values);

	for (i = 0; i < SENSOR_NUM; i++)
	{
		int value = values[i];

		if (value > ON_TRACK_VALUE)
		{
			on_track = true;
		}

		if (value > NOISE_THRESHOLD)
		{
			average += (long)(value) * heur_offset;
			sum += value;
		}

        heur_offset += HEUR_OFFSET;
	}

	/* Lost track */
	if (!on_track)
	{
		/* Last was left or center */
		if (last_value < (SENSOR_NUM - 1) * (HEUR_OFFSET >> 1))
		{
			return 0;
		}

		/* Last was right or center */
		return (SENSOR_NUM - 1) * HEUR_OFFSET;
	}

	last_value = average / sum;

	return last_value;
}

void handle_move(int values[])
{
#ifdef USART_DEBUG
	char buf[USART_BUF_LEN];
#endif

	static int last_error = 0;
	static long integral = 0;
	static int reset_integral = 0;

	if (reset_integral == INTEGRAL_RESET_ITER)
	{
		reset_integral = 0;
		integral = 0;
	}

	unsigned int position = read_qrt_line(values);
	int error = position - MIDDLE_ERROR;

	integral += error;

	int pid = KP * error + KI * integral + KD * (error - last_error);

	int left_speed = BASE_SPEED + pid;
	int right_speed = BASE_SPEED - pid;

	left_speed = clamp(left_speed, 0, MAX_SPEED);
	right_speed = clamp(right_speed, 0, MAX_SPEED);

	left_motor_set(left_speed);
	right_motor_set(right_speed);

	++reset_integral;

#ifdef USART_DEBUG
	sprintf(buf, "positon: %d\r\n", position);
	USART0_print(buf);
	sprintf(buf, "error: %d\r\n", error);
	USART0_print(buf);
	sprintf(buf, "pid: %d\r\n", pid);
	USART0_print(buf);
	sprintf(buf, "left speed: %d\r\n", left_speed);
	USART0_print(buf);
	sprintf(buf, "right speed: %d\r\n", right_speed);
    USART0_print(buf);
#endif
}


int main(void)
{
	int values[SENSOR_NUM];

#ifdef USART_DEBUG
	char buff[USART_BUF_LEN];

	/* Init usart */
	USART0_init();
#endif

	/* Init ADC */
	ADC_init();

	/* Init timers */
	engine_init();

	while (1)
	{
		read_line_input(values);

#ifdef USART_DEBUG
		sprintf(buff, "%d %d %d %d %d %d\r\n",
			values[0],
			values[1],
			values[2],
			values[3],
			values[4],
			values[5]);
		USART0_print(buff);
#endif

		handle_move(values);

		_delay_ms(SLEEP_BETWEEN);
	}

	return 0;
}
