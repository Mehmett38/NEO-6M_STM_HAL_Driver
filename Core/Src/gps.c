/*
 * md_gps.c
 *
 *  Created on: Jul 6, 2023
 *      Author: Mehmet Dincer
 */


#include "gps.h"

UART_HandleTypeDef *gpsUart;
RingBuff ringBuff;					// Storage the gps rx values
uint8_t rxGps;
GPS *_gps;

//////////////////////////////////////////////////////////////
/*
 * This function start the uart interrupt
 */
void GPS_Init(UART_HandleTypeDef *uart, GPS *gps)
{
	gpsUart = uart;
	_gps = gps;
	HAL_UART_Receive_IT(gpsUart, &rxGps , 1);
}

//////////////////////////////////////////////////////////////
/*
 * When rx pin read, store the datas in ringBuff structure
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == gpsUart->Instance)
	{
		HAL_UART_Receive_IT(gpsUart, &rxGps, 1);
		ringBuff.rxGps[HEAD_INCREASE] = rxGps;
		if(rxGps == '\r')	GPS_Parse();
	}
}

/////////////////////////////////////////////////////////////
/*
 * This function parses parameters and store them in GPS structure
 */
GPS_State GPS_Parse()
{
	while(TAIL_READ != '$')					//tail değerini $ olana kadar arttırır
	{
		if(ringBuff.tail == ringBuff.head) return NO_CONNECTION;
		TAIL_INCREASE;						//tail değerini arttırır
	}

	TAIL_INCREASE;							//$ ifadesinden sonraki kısma geçer

	if(strncmp((char*)&ringBuff.rxGps[ringBuff.tail], GPGGA_STR, 5) == 0)
	{
		return GPS_GPGGA_Parse();
	}
	else if(strncmp((char*)&ringBuff.rxGps[ringBuff.tail], GPRMC_STR, 5) == 0)
	{
		return (_gps->gpsState == POSITION_FIXED)? GPS_GPRMC_Parse() : NO_CONNECTION;
	}
	else if(strncmp((char*)&ringBuff.rxGps[ringBuff.tail], GPVTG_STR, 5) == 0)
	{
		return (_gps->gpsState == POSITION_FIXED)? GPS_GPVTG_Parse() : NO_CONNECTION;
	}
	return NO_CONNECTION;
}

/////////////////////////////////////////////////////////////
/*
 * GPRMC verilerini çözer ve gps değerine atar
 * -tarih, saat, enlem, boylam-
 */
GPS_State GPS_GPRMC_Parse()
{
	if(!GPS_CheckSumControl(&ringBuff.rxGps[ringBuff.tail]))
	{
		return WRONG_DATA;
	}

	if(ringBuff.rxGps[(ringBuff.tail + 16) % BUFF_SIZE] == 'A' && GPS_CheckSumControl(&ringBuff.rxGps[ringBuff.tail])) // okuma işlemi doğru
	{
		TAIL_INCREASE_TO_COMMA;
		//↑ ifadesi tail değerini attırır çünkü "GPRMC," ifadesi 6 karakterden oluşuyor
		_gps->hour = (TAIL_READ_P - ASCII_NUMBER) * 10 + (TAIL_READ_P - ASCII_NUMBER) + TURKIYE_UTC;
		_gps->minute = (TAIL_READ_P - ASCII_NUMBER) * 10 + (TAIL_READ_P - ASCII_NUMBER);
		_gps->second = (TAIL_READ_P - ASCII_NUMBER) * 10 + (TAIL_READ_P - ASCII_NUMBER);

		TAIL_INCREASE_TO_COMMA;
		TAIL_INCREASE_TO_COMMA;

		//genlik değerinin alır
		_gps->latitudeDegree = (TAIL_READ_P - ASCII_NUMBER) * 10 + (TAIL_READ_P - ASCII_NUMBER);
		_gps->latitudeMinute = (TAIL_READ_P - ASCII_NUMBER) * 10.0 + (TAIL_READ_P - ASCII_NUMBER);
		TAIL_INCREASE;		// "." geçtik
		_gps->latitudeSecond = ((TAIL_READ_P - ASCII_NUMBER) *100.0 + (TAIL_READ_P - ASCII_NUMBER)*10.0 + (TAIL_READ_P - ASCII_NUMBER)*1.0) / 1000 * 60;

		TAIL_INCREASE_TO_COMMA;

		// Pole bilgisini alır
		if(TAIL_READ_P == 'N')	_gps->locationLat = NORTH;
		else 					_gps->locationLat = SOUTH;

		TAIL_INCREASE_TO_COMMA;		// "," geçtik

		// Boylam bilgisini alır
		_gps->longitudeDegree = (TAIL_READ_P - ASCII_NUMBER) * 100 + (TAIL_READ_P - ASCII_NUMBER) * 10 + (TAIL_READ_P - ASCII_NUMBER);
		_gps->longitudeMinute = (TAIL_READ_P - ASCII_NUMBER) * 10.0 + (TAIL_READ_P - ASCII_NUMBER);
		TAIL_INCREASE;		// "." geçtik
		_gps->longitudeSecond = ((TAIL_READ_P - ASCII_NUMBER) * 100.0 + (TAIL_READ_P - ASCII_NUMBER) * 10.0 + (TAIL_READ_P - ASCII_NUMBER) * 1.0) / 1000 * 60;

		// yer yer bilgisi alır
		TAIL_INCREASE_TO_COMMA;
			if(TAIL_READ_P == 'E')	_gps->locationLong = EAST;
			else 					_gps->locationLong = WEST;

		TAIL_INCREASE_TO_COMMA;
		TAIL_INCREASE_TO_COMMA;
		TAIL_INCREASE_TO_COMMA;

		// Tarih bilgisini alır
		_gps->day = (TAIL_READ_P - ASCII_NUMBER) * 10 + (TAIL_READ_P - ASCII_NUMBER);
		_gps->month = (TAIL_READ_P - ASCII_NUMBER) * 10 + (TAIL_READ_P - ASCII_NUMBER);
		_gps->year = 2000 + (TAIL_READ_P - ASCII_NUMBER) * 10 + (TAIL_READ_P - ASCII_NUMBER);

		return _gps->gpsState;
	}
	else	return WRONG_DATA;
}

/////////////////////////////////////////////////////////////
/*
 * GPGGA verilerini çözer ve gps değerine atar	-uydu sayısı ve yükseklik-
 */
GPS_State GPS_GPGGA_Parse()
{
	if(!GPS_CheckSumControl(&ringBuff.rxGps[ringBuff.tail]))
	{
		return WRONG_DATA;
	}
	else
	{
		TAIL_INCREASE_TO_COMMA;					// "GPGGA," sonrasına geçer
		TAIL_INCREASE_TO_COMMA;					// "tarih sonrasına geçer
		TAIL_INCREASE_TO_COMMA;					// enlem sonrasına geçer
		TAIL_INCREASE_TO_COMMA;					// boylam sonrasına geçer
		TAIL_INCREASE_TO_COMMA;					// Boylam bölgesinin başına geçer
		TAIL_INCREASE_TO_COMMA;					// Boylam bölgesinin sonuna geçer

		if(TAIL_READ == '0')					//GNSS Position Fix Indicator
		{
			_gps->gpsState = NO_CONNECTION;
			return NO_CONNECTION;
		}
		else
			_gps->gpsState = POSITION_FIXED;

		if(TAIL_READ_P != 0)
		{
			TAIL_INCREASE_TO_COMMA;	// "," geçer
			_gps->numberOfSatellite = (TAIL_READ_P - ASCII_NUMBER) * 10 + (TAIL_READ_P - ASCII_NUMBER);

			TAIL_INCREASE_TO_COMMA;	// ",0.9" geçer
			TAIL_INCREASE_TO_COMMA;	// "," geçer

			_gps->height = (TAIL_READ_P - ASCII_NUMBER) * 100 + (TAIL_READ_P - ASCII_NUMBER) * 10 + (TAIL_READ_P - ASCII_NUMBER);
			TAIL_INCREASE;
			_gps->height += (TAIL_READ_P - ASCII_NUMBER) / 10.0;
		}
		return _gps->gpsState;
	}
}

/////////////////////////////////////////////////////////////
/*
 * GPVTG verilerini çözer ve gps değerine atar	-hız bilgisi-
 */
GPS_State GPS_GPVTG_Parse()
{
	if(!GPS_CheckSumControl(&ringBuff.rxGps[ringBuff.tail]))
	{
		return WRONG_DATA;
	}
	else
	{
		TAIL_INCREASE_TO_COMMA;		// "GPVTG,"
		TAIL_INCREASE_TO_COMMA;		//
		TAIL_INCREASE_TO_COMMA;
		TAIL_INCREASE_TO_COMMA;
		TAIL_INCREASE_TO_COMMA;
		TAIL_INCREASE_TO_COMMA;
		TAIL_INCREASE_TO_COMMA;

		uint8_t tailValue1 = ringBuff.tail;
		TAIL_INCREASE_DOT;
		uint8_t tailValue2 = ringBuff.tail;

		signed char counter= tailValue2 - tailValue1 - 2;
		ringBuff.tail = tailValue1;
		_gps->speed = 0;
		for(;counter >= 0; counter--)
		{
			_gps->speed += (TAIL_READ - ASCII_NUMBER) * (uint32_t)pow(10,counter);
		}
		ringBuff.tail = tailValue2;

		_gps->speed += (TAIL_READ_P - ASCII_NUMBER) / 10.0 + (TAIL_READ_P - ASCII_NUMBER) / 100.0 + (TAIL_READ_P - ASCII_NUMBER) / 1000.0;

		return _gps->gpsState;
	}
}

/*
 * tüm değerleri XOR işlemine tabi tutarak bir checksum değeri üretir
 */
uint8_t GPS_CheckSumControl(uint8_t *ptr)
{
	int i = 0;
	int check = 0;
	char checHexa[5];

	while(ptr[i] != '*')
	{
		check ^= ptr[i++];
	}
	sprintf(checHexa,"%x",check);
	return !strncmp((char*)&ptr[++i], checHexa,2);
}



