/*
 * md_gps.h
 *
 *  Created on: Jul 6, 2023
 *      Author: Mehmet Dincer
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include <main.h>
#include <stm32f4xx_hal.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


#define BUFF_SIZE					(250UL)
#define ASCII_NUMBER				(48)
#define FIRST_FIXED					(1)
#define TURKIYE_UTC					(3)


#define GPRMC_STR					("GPRMC")		//Recommended Minimum Specific GNSS Data
#define GPGLL_STR					("GPGLL")		//Geographic Position - Latitude/Longitude
#define GPGGA_STR					("GPGGA")		//Global Positioning System Fixed Data
#define GPVTG_STR					("GPVTG")		//Course Over Ground and Ground Speed
#define GPZDA_STR					("GPZDA")		//SiRF Timing Message

typedef enum{
	NO_CONNECTION,
	WRONG_DATA,
	POSITION_FIXED,
}GPS_State;

typedef enum{
	NORTH,
	SOUTH,
	EAST,
	WEST,
}Location;

/* Ring buffer structure*/
typedef struct{
	uint8_t rxGps[BUFF_SIZE];
	volatile uint8_t tail;
	volatile uint8_t head;
}RingBuff;

/* GPS structure*/
typedef struct{
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint8_t latitudeDegree;			//enlem derecesi
	uint8_t latitudeMinute;
	float latitudeSecond;
	Location locationLat;
	uint8_t longitudeDegree;		//boylam
	uint8_t longitudeMinute;
	float longitudeSecond;
	Location locationLong;
	uint8_t day;
	uint8_t month;
	uint16_t year;
	uint8_t numberOfSatellite;
	float height;
	float speed;
	GPS_State gpsState;
}GPS;


extern RingBuff ringBuff;

#define HEAD_INCREASE				(ringBuff.head = ((ringBuff.head + 1) % BUFF_SIZE))
#define TAIL_READ					(ringBuff.rxGps[ringBuff.tail % BUFF_SIZE])		// tail değerini okur
#define TAIL_READ_P					(ringBuff.rxGps[ringBuff.tail++ % BUFF_SIZE])	// tail değerini okur ve taili arttırır
#define TAIL_INCREASE				(ringBuff.tail = ((ringBuff.tail + 1) % BUFF_SIZE))				// tail değerini arttırır
#define TAIL_INCREASE_N(x)			(ringBuff.tail = ((ringBuff.tail + x) % BUFF_SIZE))
#define TAIL_INCREASE_TO_COMMA			do{\
									}while(TAIL_READ_P != ',')

#define TAIL_INCREASE_DOT			do{\
									}while(TAIL_READ_P != '.')


void GPS_Init(UART_HandleTypeDef *uart, GPS *gps);
GPS_State GPS_Parse();
GPS_State GPS_GPRMC_Parse();
GPS_State GPS_GPGGA_Parse();
GPS_State GPS_GPVTG_Parse();
uint8_t GPS_CheckSumControl(uint8_t *ptr);

#endif /* INC_GPS_H_ */
