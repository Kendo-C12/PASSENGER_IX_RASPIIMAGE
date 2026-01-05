#include <Arduino.h>
#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3

SFE_UBLOX_GNSS max10s; 

constexpr uint32_t UBLOX_CUSTOM_MAX_WAIT = 250ul;
const int apogee_size = 255;
char apogee_packet[255];
int lenChunk;

uint32_t gps_time;

double latitude = 1.111111;
double longitude = 1.111111;
float altitude = 1.111111;

void setup()
{
    Serial.begin(115200);
    delay(1000); 
    Serial.println("SparkFun u-blox Example");

    Wire.begin(); 

    while (max10s.begin() == false) //Connect to the u-blox module using Wire port
    {
        Serial.println(F("u-blox GNSS not detected at default I2C address. Retrying..."));
        delay (1000);
    }

    max10s.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    max10s.setNavigationFrequency(25, VAL_LAYER_RAM_BBR,UBLOX_CUSTOM_MAX_WAIT);
    max10s.setAutoPVT(true, VAL_LAYER_RAM_BBR,UBLOX_CUSTOM_MAX_WAIT);
    max10s.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR,UBLOX_CUSTOM_MAX_WAIT);

    gps_time = millis();
}


//   if(millis() > gps_time){
//     gps_time = millis() + 2000;
//     Serial.println("try");
    // if (max10s.getPVT() == true)
    // {
    //     latitude  = static_cast<double>(max10s.getLatitude()) * 1.e-7;
    //     longitude = static_cast<double>(max10s.getLongitude()) * 1.e-7;
    //     altitude  = static_cast<float>(max10s.getAltitudeMSL()) * 1.e-3f;

    //     Serial.print(F("Lat: "));
    //     Serial.print(latitude);

    //     Serial.print(F(" Long: "));
    //     Serial.print(longitude);
    //     Serial.print(F(" (degrees * 10^-7)"));

    //     Serial.print(F(" Alt: "));
    //     Serial.print(altitude);
    //     Serial.print(F(" (mm)"));

        // uint8_t hour = max10s.getHour();
        // Serial.print(hour);
        // Serial.print(":");
        // uint8_t minute = max10s.getMinute();
        // Serial.print(minute);
        // Serial.print(":");
        // uint8_t second = max10s.getSecond();
        // Serial.print(second);
    // }
void loop()
{
  if(millis() > gps_time){
    gps_time = millis() + 2000;
    Serial.println("try");
    
    uint32_t hour = 32;
    uint32_t min = 16;
    uint32_t sec = 8;

    char hour_str[16];
    char min_str[16];
    char sec_str[16];

    
    String s;

    sprintf(hour_str, "%lu", hour);  // %lu for uint32_t (unsigned long)
    sprintf(min_str, "%lu", min);
    sprintf(sec_str, "%lu", sec);
    
    
    snprintf(apogee_packet, sizeof(apogee_packet), "GS,%s:%s:%s", hour_str, min_str, sec_str);
    Serial.println(apogee_packet);
  }
}