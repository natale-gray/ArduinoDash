#include <stdio.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>



#define MAX_LEFT_SA_SENSOR 40
#define MAX_LEFT_SA_WHEEL -23
#define MAX_RIGHT_SA_SENSOR 120
#define MAX_RIGHT_SA_WHEEL 23

#define NUM_LEDS 17
#define SA_PIN A5 
#define FLWS_PIN A4
#define RLWS_PIN A3
#define IS_PIN 3
#define LED_PIN 6
// #define CANRX 13 
#define DRIVESIGNAL_PIN 12
// #define CANTX 10
#define BUZZER_PIN 9
#define BP_PIN A2

const uint16_t battery_health_red[9] = {(255/11)*8, (255/11)*7, (255/11)*6, (255/11)*5, (255/11)*4, (255/11)*3, (255/11)*2, (255/11)*1, (255/11)*0};
const uint16_t battery_health_green[9] = {(255/11)*3, (255/11)*4, (255/11)*5, (255/11)*6, (255/11)*7, (255/11)*8, (255/11)*9, (255/11)*10, (255/11)*11};

Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);


typedef struct {
    bool AMS_Fault;
    bool BSPD_Fault;
    bool IMD_Fault;
    bool PUMP_Indicator;
    bool FAN_Indicator;
    bool Launch_Control;
    bool Brake_Status;
    bool Miscellaneous;
} Flags;

typedef struct {
    uint8_t AMS_Indicator_Index;
    uint8_t BSPD_Indicator_Index;
    uint8_t IMD_Indicator_Index;
    uint8_t PUMP_Indicator_Index;
    uint8_t FAN_Indicator_Index;
    uint8_t Launch_Control_Indicator_Index;
    uint8_t Brake_Status_Indicator_Index;
    uint8_t Miscellaneous_Indicator_Index;
} Flags_Indexes;

Flags Indicator_Flags = {true, 
                         true, 
                         false, 
                         true,
                         true,
                         false,
                         true,
                         true
                         };  // is_enabled = true, is_visible = false, is_editable = true

Flags_Indexes Indicator_Indexes = {4,
                                   3,
                                   2,
                                   1,
                                   0,
                                   15,
                                   14,
                                   16
                                   };
                                   
                                   


void UpdateIndicators(void) {
  if (Indicator_Flags.AMS_Fault) {
    pixels.setPixelColor(Indicator_Indexes.AMS_Indicator_Index, pixels.Color(255, 0, 0));
  }
  else pixels.setPixelColor(Indicator_Indexes.AMS_Indicator_Index, pixels.Color(0, 30, 0));

  if (Indicator_Flags.BSPD_Fault) {
    pixels.setPixelColor(Indicator_Indexes.BSPD_Indicator_Index, pixels.Color(255, 0, 0));
  }
  else pixels.setPixelColor(Indicator_Indexes.BSPD_Indicator_Index, pixels.Color(0, 30, 0));

  if (Indicator_Flags.IMD_Fault) {
    pixels.setPixelColor(Indicator_Indexes.IMD_Indicator_Index, pixels.Color(255, 0, 0));
  }
  else pixels.setPixelColor(Indicator_Indexes.IMD_Indicator_Index, pixels.Color(0, 30, 0));

  if (Indicator_Flags.PUMP_Indicator) {
    pixels.setPixelColor(Indicator_Indexes.PUMP_Indicator_Index, pixels.Color(255, 0, 0));
  }
  else pixels.setPixelColor(Indicator_Indexes.PUMP_Indicator_Index, pixels.Color(0, 30, 0));

  if (Indicator_Flags.FAN_Indicator) {
    pixels.setPixelColor(Indicator_Indexes.FAN_Indicator_Index, pixels.Color(255, 0, 0));
  }
  else pixels.setPixelColor(Indicator_Indexes.FAN_Indicator_Index, pixels.Color(0, 30, 0));

  if (Indicator_Flags.Launch_Control) {
    pixels.setPixelColor(Indicator_Indexes.Launch_Control_Indicator_Index, pixels.Color(255, 0, 0));
  }
  else pixels.setPixelColor(Indicator_Indexes.Launch_Control_Indicator_Index, pixels.Color(0, 30, 0));

  if (Indicator_Flags.Brake_Status) {
    pixels.setPixelColor(Indicator_Indexes.Brake_Status_Indicator_Index, pixels.Color(255, 0, 0));
  }
  else pixels.setPixelColor(Indicator_Indexes.Brake_Status_Indicator_Index, pixels.Color(0, 30, 0));

  if (Indicator_Flags.Miscellaneous) {
    pixels.setPixelColor(Indicator_Indexes.Miscellaneous_Indicator_Index, pixels.Color(255, 0, 0));
  }
  else pixels.setPixelColor(Indicator_Indexes.Miscellaneous_Indicator_Index, pixels.Color(0, 30, 0));

  
}

void UpdateBatteryHealth(uint8_t bat) {

  // Highlight the red ones:
  for (int i=5; i<13 - bat; i++) {
    pixels.setPixelColor(i, pixels.Color(255, 0, 0));
  }
  Serial.println("LED CONTROL SEQUENCE 3");

  // highlight the green ones
  for (int i=0; i<=bat; i++) {
    pixels.setPixelColor(13-i, pixels.Color(battery_health_red[bat], battery_health_green[bat], 0));
  }
  
  pixels.show();
  delay(2000);
}




void PlayRTDBuzzer(uint8_t pin_num) {
    digitalWrite(pin_num, HIGH);

    delay(1000); // Delay for 1000 milliseconds (1 second)

    digitalWrite(pin_num, LOW);
    delay(1000);

}