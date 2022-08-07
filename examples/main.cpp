#include <Wire.h>
#include "AmbasatSAM_M8Q.h"

AmbasatSAM_M8Q *ambasat_m8q;
int lastTime;


void setup(void)
{
   Serial.begin(9800); // console
    // initialize the UBLOX M8Q GNSS sensor
    ambasat_m8q = new AmbasatSAM_M8Q();  
    for(int i=0; i< 5; i++)
    {
        if(ambasat_m8q->begin() == E_FAIL)
        {
            delay(3000);
        }
        else
        {
            break;
        } 
    }
    
    if(E_FAIL == ambasat_m8q->setCommunication(UBOX_ONLY,NMEA_ONLY))
    {
        Serial.println("Failing setting communication mode");
    } 
    lastTime=0;
}

void loop()
{
    if (millis() - lastTime > 1000)
    {   
        error_t status = ambasat_m8q->read_data();
        if(E_SUCCESS == status )
        {
            digitalWrite(LED,HIGH);
            if(ambasat_m8q->field_data.fix_type > 0)
            {
                Serial.print("LATITUDE: ");
                Serial.println(ambasat_m8q->field_data.latitude* 0.000001,6);
                Serial.print("LONGITUDE: ");
                Serial.println(ambasat_m8q->field_data.longitude* 0.000001,6);
                Serial.print("Altitude(m): ");
                Serial.println(ambasat_m8q->field_data.altitude/1000.0);
                Serial.print("#N. Satellites: ");
                Serial.println(ambasat_m8q->field_data.num_satellites);
                Serial.print("SPEED(m/s): ");
                Serial.println(ambasat_m8q->field_data.speed/1000.0);
                Serial.print("Fix Type: ");
                Serial.println(ambasat_m8q->field_data.fix_type);
            }
            else
            {
                Serial.println("No FIX");
            }
        }
        else
        {
             Serial.println("Error sensor busy");
        }
        lastTime=millis();
    }
}