/*******************************************************************************
* AmbaSat-1
* Filename: AmbasatBME680.h
*
* This library is designed for use with AmbaSat-1 and is a wrapper for
* BME680 sensor
* 
* Copyright (c) 2022 AmbaSat Ltd
* https://ambasat.com
*
* licensed under Creative Commons Attribution-ShareAlike 3.0
* ******************************************************************************/

#ifndef __AmbasatSAM_M8Q__
#define __AmbasatSAM_M8Q__

#include "Arduino.h"
#include <Wire.h>
#include "debugging.h"
#include "MicroNMEA.h"


#define DEFAULT_I2C_ADDRESS 0x42
#define N_BYTES_AVAILABLE_H 0xFD
#define N_BYTES_AVAILABLE_L 0xFE
#define MAX_MESSAGE_LENGTH 256
#define DATA_STREAM 0xFF
#define SYNC_CHAR_1 0xB5
#define SYNC_CHAR_2 0x62
#define NAV_CLASS 0x01
#define RXM_CLASS 0x02
#define INF_CLASS 0x04
#define ACK_CLASS 0x05
#define CFG_CLASS 0x06
#define UPD_CLASS 0x09
#define MON_CLASS 0x0A
#define AID_CLASS 0x0B
#define TIM_CLASS 0x0D
#define ESF_CLASS 0x10
#define MGA_CLASS 0x13
#define LOG_CLASS 0x21
#define SEC_CLASS 0x27
#define HNR_CLASS 0x28

#define ACK 0x01
#define NAK 0x00

//********* CFG MESSAGE SECTION **********
#define CFG_PRT 0x00
#define CFG_MSG 0x01
#define CFG_RATE 0x08

/*!
 * @brief Sensor field data structure
 */
typedef struct fields {
	/*  */
	uint8_t fix_type;
	/* */
	long altitude;
	/*  */
	double latitude;
	/*!  */
	double longitude;
	/*! */
	long nav_system;
	/*!  */
	uint8_t num_satellites;
	/*!  */
	int32_t speed;

}field_data_t;

/*!
 * @brief Ublox message data structure
 */
typedef struct message{
	uint8_t sync_A;
	uint8_t sync_B;
	uint8_t class_ID;
	uint8_t message_ID;
	uint16_t payload_lenght;
	uint8_t payload[MAX_MESSAGE_LENGTH];
	uint8_t checksum_A;
	uint8_t checksum_B;
}ublox_message_t;


enum _conf_protocol
{
    NMEA_ONLY=0x02,
    UBOX_ONLY=0x01,
    NMEA_UBOX=0x03,
};

class AmbasatSAM_M8Q{
    public:
        AmbasatSAM_M8Q(); 
        error_t begin();
        error_t read_data();
		error_t setCommunication(_conf_protocol input, _conf_protocol output);
		field_data_t field_data;
		error_t waitForUbxMessage(ublox_message_t *msg, uint32_t timeoutMillis, uint32_t intervalMillis);
		error_t power_Off(uint32_t sleep_time);
    private:
		void reset_payload(ublox_message_t &msg);
		error_t readUBX(ublox_message_t *new_msg);
		uint16_t get_available_bytes();
		error_t parse_navigation_data(ublox_message_t ublox_msg);
		void checksum_calculator(ublox_message_t *msg);
		error_t writeUbxMessage(ublox_message_t *msg);
		uint32_t extract4U(uint8_t *payload_field, uint8_t index);
		error_t readNMEA();
};

#endif // __AmbasatBME680__