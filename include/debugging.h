/*******************************************************************************
* AmbaSat-1 
* Filename: Debugging.h
*
* This file provides debugging support for AmbaSat-1 and its I2C sensor boards
* 
* Copyright (c) 2021 AmbaSat Ltd
* https://ambasat.com
*
* licensed under Creative Commons Attribution-ShareAlike 3.0
* ******************************************************************************/

#ifndef __Debugging__
#define __Debugging__

#define DEBUGGING 

#ifdef DEBUGGING
    #define PRINT_DEBUG(x)    Serial.print(x)
    #define PRINTLN_DEBUG(x)  Serial.println(x)
    #define PRINT_HEX(x)      Serial.print(x, HEX)
#else
    #define PRINT_DEBUG(x)    
    #define PRINTLN_DEBUG(x)  
    #define PRINT_HEX(x)      
#endif


enum _config_error_codes
{
    E_FAIL=0,
    E_SUCCESS,
    E_UNDEFINED,
    E_INVALID_INPUT,
    E_SENSOR_NOT_RESPONDING,
    E_SENSOR_I2C_FAIL,
    E_SENSOR_BUSY,
    E_SENSOR_FREE,
    E_NOT_IN_SYNC,
};


typedef enum _config_error_codes error_t;


#endif  // __Debugging__
