/*******************************************************************************
* AmbaSat-1 
* Filename: AmbasatSAM_M8Q.cpp
*
* This library is designed specifically for AmbaSat-1 and the sensor-8 Ublox-M8Q
* 
* Copyright (c) 2022 AmbaSat Ltd
* https://ambasat.com
*
* licensed under Creative Commons Attribution-ShareAlike 3.0
* ******************************************************************************/

#include "AmbasatSAM_M8Q.h"

char nmeaBuffer[100];
char buff[32];
int idx = 0;
_conf_protocol output_protocol;
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

// ======================================================================================
AmbasatSAM_M8Q::AmbasatSAM_M8Q() 
{

}

// ======================================================================================
/*! @brief  Initialize the sensor Ublox-M8Q
* 
* @return E_SUCCESS in case initialization is successful, E_FAIL otherwise
*/
error_t AmbasatSAM_M8Q::begin() {
    Wire.begin();            
    Wire.beginTransmission(DEFAULT_I2C_ADDRESS);     
    if (Wire.endTransmission() != 0) { 
        return E_FAIL;
    }
    // reset the fields 
    field_data.altitude=0;
    field_data.latitude=0;
    field_data.fix_type=0;
    field_data.speed=0;
    field_data.num_satellites=0;
    return E_SUCCESS;
} 

/*! @brief  Poll the sensor and request GSNN data using the NMEA or UBOX protocol
* 
* @return E_SUCCESS in case of success, E_SENSOR_NOT_RESPONDING otherwise
*/
error_t AmbasatSAM_M8Q::read_data()
{
    // read data using the NMEA protocol 
    if (output_protocol == NMEA_ONLY)
    {
        error_t status = readNMEA();
        if(E_SUCCESS != status)
        {
            return status;
        }
        if(nmea.isValid())
        {
            field_data.fix_type=1;
            field_data.latitude= nmea.getLatitude();
            field_data.longitude= nmea.getLongitude();
            nmea.getAltitude(field_data.altitude);
            field_data.speed=nmea.getSpeed();
            field_data.num_satellites=nmea.getNumSatellites();
        }
        else
        {
            field_data.fix_type=0;
        }
        return E_SUCCESS;   
    }
    // read data using the UBOX protocol (STILL NOT RELIABLE)
    else if(output_protocol == UBOX_ONLY)
    {
        ublox_message_t ublox_msg;
        ublox_msg.class_ID= NAV_CLASS;
        ublox_msg.message_ID= 0x07;
        ublox_msg.payload_lenght=0;
        writeUbxMessage(&ublox_msg);
        if(waitForUbxMessage(&ublox_msg,1000,50) == E_SUCCESS)
        {
            parse_navigation_data(ublox_msg);
            return E_SUCCESS;
        }
    }   
    return E_FAIL;
}

/*! @brief Return the number of bytes available to read
* 
* @return uint16_t > 0, 0 otherwise (even in case of error)
*/
uint16_t AmbasatSAM_M8Q::get_available_bytes(){

    Wire.beginTransmission(DEFAULT_I2C_ADDRESS);
    Wire.write(N_BYTES_AVAILABLE_H);
    if (Wire.endTransmission(false) != 0)
    {
        return 0;
    }
    Wire.requestFrom(DEFAULT_I2C_ADDRESS, 2);
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    // no data on the buffer
    if (msb == 0xFF || lsb == 0xFF)
    {
        return 0;
    }
    msb &= 0x7F;
    return ((uint16_t) msb << 8 | lsb);
}

/*! @brief  Wait for a specific UBX message from the GPS
* @param msg data structe for the message
* @param timeoutMillis timout time in millisecond 
* @param intervalMillis deleay for interval in millisecond
* @return E_SUCCESS if the message is correctly read, E_FAIL otherwise
*/
error_t AmbasatSAM_M8Q::waitForUbxMessage(ublox_message_t *msg, uint32_t timeoutMillis, uint32_t intervalMillis){
    unsigned long startTime = millis();
    uint8_t desiredClass = msg->class_ID;
    uint8_t desiredId = msg->message_ID;

    uint32_t currTime = startTime;
    while (currTime - startTime < timeoutMillis)
    {
        error_t status = readUBX(msg);
        if (status == E_SUCCESS) 
        {
            if (msg->class_ID== desiredClass && msg->message_ID == desiredId)
                return E_SUCCESS;
        }
        delay(intervalMillis);
        currTime = millis();
    }
    return E_FAIL;
}

/*! @brief  Read NMEA values over I2C
* 
* @return E_SUCCESS if the message is correctly read, E_SENSOR_NOT_RESPONDING if the sensor is not responding
*/
error_t AmbasatSAM_M8Q::readNMEA()
{
    uint16_t bytes= get_available_bytes();
    Wire.beginTransmission(DEFAULT_I2C_ADDRESS);
    Wire.write((uint8_t) DATA_STREAM);
    if (Wire.endTransmission(false) != 0)
    {
        return E_SENSOR_NOT_RESPONDING;
    }
    if(bytes > 32)
        Wire.requestFrom((uint8_t)DEFAULT_I2C_ADDRESS, (uint8_t) 32);
    else
        Wire.requestFrom((uint8_t)DEFAULT_I2C_ADDRESS, (uint8_t) bytes);
    while (Wire.available())
    {
        char c = Wire.read();
        delay(1);
        // reject the character if is not valid
        if ((uint8_t) c != 0xFF)
        {
            nmea.process(c);
        }
    }
   return E_SUCCESS;
}

/*! @brief  Read UBX message values over I2C (NOT SAFE TO USE YET)
*/
error_t AmbasatSAM_M8Q::readUBX(ublox_message_t *new_msg){
    uint16_t bytes_to_read = get_available_bytes();
   
    uint8_t lenght_M,lenght_L;
    Wire.beginTransmission(DEFAULT_I2C_ADDRESS);
    Wire.write(DATA_STREAM);
    if (Wire.endTransmission(false) != 0)
    {
        return E_SENSOR_NOT_RESPONDING;
    }
    if (bytes_to_read > 32)
    {
        Wire.requestFrom(DEFAULT_I2C_ADDRESS, 32);
    }
    else
    {
        Wire.requestFrom(DEFAULT_I2C_ADDRESS, bytes_to_read);
    }
    uint8_t buf_size=0;
    if (Wire.available()){
       
        new_msg->sync_A = Wire.read();
        new_msg->sync_B = Wire.read();
       
        if (!(new_msg->sync_A == SYNC_CHAR_1 && new_msg->sync_B == SYNC_CHAR_2))
            return E_NOT_IN_SYNC;

        new_msg->class_ID=Wire.read();
        new_msg->message_ID=Wire.read();
        lenght_L=Wire.read();
        lenght_M=Wire.read();
        new_msg->payload_lenght= lenght_M << 8 | lenght_L;
      
        buf_size+=6;
        for(uint16_t i=0; i< new_msg->payload_lenght; i++)
        {
            new_msg->payload[i] = Wire.read();
            buf_size++;
            if (buf_size >= 32)
            {
                bytes_to_read-=buf_size;
                if (bytes_to_read > 32)
                {
                    Wire.requestFrom(DEFAULT_I2C_ADDRESS, 32);
                }
                else
                {
                     Wire.requestFrom(DEFAULT_I2C_ADDRESS, bytes_to_read);
                }
                buf_size=0;
            }
        }
        new_msg->checksum_A = Wire.read();
        new_msg->checksum_B = Wire.read();
        return E_SUCCESS;       
    }
    return E_FAIL;
}

/*! @brief  Send a UBX message to the sensor
* @param msg data structe for the message
*
* @return E_SUCCESS if the message is correctly read, E_SENSOR_I2C_FAIL otherwise
*/
error_t AmbasatSAM_M8Q::writeUbxMessage(ublox_message_t *msg){
    checksum_calculator(msg);
    Wire.beginTransmission(DEFAULT_I2C_ADDRESS);
    if (Wire.endTransmission(false) != 0)
        return E_SENSOR_I2C_FAIL;

    Wire.beginTransmission(DEFAULT_I2C_ADDRESS);
    Wire.write(SYNC_CHAR_1);
    Wire.write(SYNC_CHAR_2);
    Wire.write(msg->class_ID);
    Wire.write(msg->message_ID);
    Wire.write((uint8_t) (msg->payload_lenght & 0xFF)); // length lsb
    Wire.write((uint8_t) (msg->payload_lenght >> 8)); // length msb

    for (uint8_t i = 0; i < msg->payload_lenght; i++)
        Wire.write(msg->payload[i]);

    Wire.write(msg->checksum_A);
    Wire.write(msg->checksum_B);

    Wire.endTransmission();

    return E_SUCCESS;
}


/*! @brief  Reset payload to all 0's
* @param msg payload to be reseted
*
*/
void AmbasatSAM_M8Q::reset_payload(ublox_message_t &msg){
    for (int i = 0; i < msg.payload_lenght; i++)
    msg.payload[i] = 0;
}


/*! @brief  Sets the communication protocol for the input/output port of the GPS sensor
* @param input input configuration
* @param output output configuration
*
* @return E_SUCCESS if the message is correctly read, E_SENSOR_I2C_FAIL otherwise
*/
error_t AmbasatSAM_M8Q::setCommunication(_conf_protocol input, _conf_protocol output){
    // prepare payload
    ublox_message_t new_msg;
    new_msg.class_ID= CFG_CLASS;
    new_msg.message_ID= CFG_PRT;
    new_msg.payload_lenght=20;
    reset_payload(new_msg);
    new_msg.payload[4] = 0x84;
    new_msg.payload[12] = input;
    new_msg.payload[14] = output;
    writeUbxMessage(&new_msg);
    output_protocol=output;
    return E_SUCCESS;
}

/*! @brief  Calculates the checksum for UBX packets
* @param msg pointer to the packet for which we want to calculate checksums
*
*/
void AmbasatSAM_M8Q::checksum_calculator(ublox_message_t *msg)
{
    uint8_t ck_A=0; uint8_t ck_B=0;
    
    ck_A = msg->class_ID;
    ck_B = ck_A;
    ck_A += msg->message_ID;
    ck_B += ck_A;

    ck_A += msg->payload_lenght % (1 <<8);
    ck_B+=ck_A;

    ck_A += msg->payload_lenght >> 8;
    ck_B+=ck_A;

    for(uint16_t i=0; i < msg->payload_lenght; i++)
    {
        ck_A += msg->payload[i];
        ck_B += ck_A;
    }
    msg->checksum_A=ck_A;
    msg->checksum_B=ck_B;
}

/*! @brief Parse navigation data when using UBLOX protocol
* @param ublox_msg ublox message to be parsed
*
*/
error_t AmbasatSAM_M8Q::parse_navigation_data(ublox_message_t ublox_msg)
{
    field_data.fix_type=ublox_msg.payload[20];    
    field_data.latitude= (int32_t) extract4U(ublox_msg.payload,28);
    field_data.longitude= (int32_t) extract4U(ublox_msg.payload,24);
    field_data.speed= extract4U(ublox_msg.payload,60);
    field_data.num_satellites=ublox_msg.payload[23];
    field_data.altitude = extract4U(ublox_msg.payload,36);
    //field_data.horizontal_accuracy= extract4U(ublox_msg.payload,40);
    //field_data.vertical_accuracy=extract4U(ublox_msg.payload,44);
    //field_data.speed_accuracy=extract4U(ublox_msg.payload,68);
}

/*! @brief  Extract 4 byte of data from a payload
* @param payload_field message payload
* @param index start index of the data to be read
*
* @return E_SUCCESS if the message is correctly read, E_SENSOR_I2C_FAIL otherwise
*/
uint32_t AmbasatSAM_M8Q::extract4U(uint8_t *payload_field, uint8_t index)
{
    uint16_t field_1 = ((uint16_t) payload_field[index] | ((uint16_t) payload_field[index+1]) << 8);
    uint16_t field_2 = ((uint16_t) payload_field[index+2] | ((uint16_t) payload_field[index+3]) << 8);
    return (uint32_t) field_1 | ((uint32_t) field_2) << 16;
}


/*! @brief  Put the GPS sensors into sleep mode
* @param sleep_time duration of sleep in millisecond
*
* @return E_SUCCESS if the sensor is powered  off , E_SENSOR_I2C_FAIL otherwise
*/
error_t AmbasatSAM_M8Q::power_Off(uint32_t sleep_time)
{
    ublox_message_t new_msg;
    new_msg.class_ID= 0x02;
    new_msg.message_ID= 0x41;
    new_msg.payload_lenght=8;
    reset_payload(new_msg);
    new_msg.payload[0] = (sleep_time >> (8 * 0)) & 0xff;
    new_msg.payload[1] = (sleep_time >> (8 * 1)) & 0xff;
    new_msg.payload[2] = (sleep_time >> (8 * 2)) & 0xff;
    new_msg.payload[3] = (sleep_time >> (8 * 3)) & 0xff;
    new_msg.payload[4] = 0x02;
    new_msg.payload[5] = 0x00;
    new_msg.payload[6] = 0x00;
    new_msg.payload[7] = 0x00;
    writeUbxMessage(&new_msg);
    return E_SUCCESS;
}