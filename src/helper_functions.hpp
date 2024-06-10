#include <stdio.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <Arduino_CAN.h>

#define NUM_LEDS 17
#define SA_PIN A5 
#define FLWS_PIN A4
#define RLWS_PIN A3
#define IS_PIN 3
#define LED_PIN 6
#define DRIVESIGNAL_PIN 12
#define BUZZER_PIN 9
#define BP_PIN A2
#define BSPD_PIN 7
#define MISC_PIN 8

#define ADC_MAX_VAL ((1 << 14) - 1)

const float vref = 5.0;

const uint16_t SOC_Red_Colors[9] {(255/11)*8, 
                                  (255/11)*7,
                                  (255/11)*6,
                                  (255/11)*5,
                                  (255/11)*4,
                                  (255/11)*3,
                                  (255/11)*2,
                                  (255/11)*1,
                                  (255/11)*0,
                                  };
const uint16_t SOC_Green_Colors[9] {(255/11)*3,
                                  (255/11)*4,
                                  (255/11)*5,
                                  (255/11)*6,
                                  (255/11)*7,
                                  (255/11)*8,
                                  (255/11)*9,
                                  (255/11)*10,
                                  (255/11)*11
                                  } ;

Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
typedef struct {
  uint16_t Off[3];
  uint16_t Red[3];
  uint16_t Green[3];
  uint16_t Blue[3];
  uint16_t Yellow[3];
  uint16_t Purple[3];
} Colors;

Colors fault_indicator_colors = {
  {0, 0, 0}, 
  {255, 0, 0}, 
  {0, 255, 0}, 
  {0, 0, 255}, 
  {255, 255, 0}, 
  {160, 32, 240}
};

typedef struct {
    bool AMS_Fault;
    bool APPS_Fault;
    bool BSPD_Fault;
    bool IMD_Fault;
    bool Cooling_Indicator;
    bool Launch_Control;
    bool Brake_Status;
    bool Miscellaneous;
} Flags;

typedef struct {
    uint8_t AMS_Indicator_Index;
    uint8_t APPS_Indicator_Index;
    uint8_t BSPD_Indicator_Index;
    uint8_t IMD_Indicator_Index;
    uint8_t Cooling_Indicator_Index;
    uint8_t Launch_Control_Indicator_Index;
    uint8_t Brake_Status_Indicator_Index;
    uint8_t Miscellaneous_Indicator_Index;
} Flags_Indexes;

Flags Indicator_Flags = {false, 
                         false,
                         false, 
                         false, 
                         false,
                         false,
                         false,
                         false,
                         };  // is_enabled = true, is_visible = false, is_editable = true

Flags_Indexes Indicator_Indexes = {14, //
                                   15,
                                   16,
                                   1,
                                   0,
                                   2,
                                   3,
                                   4,
                                   };
                                   

void UpdateIndicators(void) {
  if (Indicator_Flags.AMS_Fault) {
    pixels.setPixelColor(Indicator_Indexes.AMS_Indicator_Index, pixels.Color(fault_indicator_colors.Red[0], fault_indicator_colors.Red[1], fault_indicator_colors.Red[2]));
  }
  else pixels.setPixelColor(Indicator_Indexes.AMS_Indicator_Index, pixels.Color(fault_indicator_colors.Off[0], fault_indicator_colors.Off[1], fault_indicator_colors.Off[2]));

  if (Indicator_Flags.APPS_Fault) {
    pixels.setPixelColor(Indicator_Indexes.APPS_Indicator_Index, pixels.Color(fault_indicator_colors.Yellow[0], fault_indicator_colors.Yellow[1], fault_indicator_colors.Yellow[2]));
  }
  else pixels.setPixelColor(Indicator_Indexes.APPS_Indicator_Index, pixels.Color(fault_indicator_colors.Off[0], fault_indicator_colors.Off[1], fault_indicator_colors.Off[2]));

  if (Indicator_Flags.BSPD_Fault) {
    pixels.setPixelColor(Indicator_Indexes.BSPD_Indicator_Index, pixels.Color(fault_indicator_colors.Red[0], fault_indicator_colors.Red[1], fault_indicator_colors.Red[2]));
  }
  else pixels.setPixelColor(Indicator_Indexes.BSPD_Indicator_Index, pixels.Color(fault_indicator_colors.Off[0], fault_indicator_colors.Off[1], fault_indicator_colors.Off[2]));

  if (Indicator_Flags.IMD_Fault) {
    pixels.setPixelColor(Indicator_Indexes.IMD_Indicator_Index, pixels.Color(fault_indicator_colors.Red[0], fault_indicator_colors.Red[1], fault_indicator_colors.Red[2]));
  }
  else pixels.setPixelColor(Indicator_Indexes.IMD_Indicator_Index, pixels.Color(fault_indicator_colors.Off[0], fault_indicator_colors.Off[1], fault_indicator_colors.Off[2]));

  if (Indicator_Flags.Cooling_Indicator) {
    pixels.setPixelColor(Indicator_Indexes.Cooling_Indicator_Index, pixels.Color(fault_indicator_colors.Blue[0], fault_indicator_colors.Blue[1], fault_indicator_colors.Blue[2]));
  }
  else pixels.setPixelColor(Indicator_Indexes.Cooling_Indicator_Index, pixels.Color(fault_indicator_colors.Off[0], fault_indicator_colors.Off[1], fault_indicator_colors.Off[2]));

  if (Indicator_Flags.Launch_Control) {
    pixels.setPixelColor(Indicator_Indexes.Launch_Control_Indicator_Index, pixels.Color(fault_indicator_colors.Purple[0], fault_indicator_colors.Purple[1], fault_indicator_colors.Purple[2]));
  }
  else pixels.setPixelColor(Indicator_Indexes.Launch_Control_Indicator_Index, pixels.Color(fault_indicator_colors.Off[0], fault_indicator_colors.Off[1], fault_indicator_colors.Off[2]));

  if (Indicator_Flags.Brake_Status) {
    pixels.setPixelColor(Indicator_Indexes.Brake_Status_Indicator_Index, pixels.Color(fault_indicator_colors.Red[0], fault_indicator_colors.Red[1], fault_indicator_colors.Red[2]));
  }
  else pixels.setPixelColor(Indicator_Indexes.Brake_Status_Indicator_Index, pixels.Color(fault_indicator_colors.Off[0], fault_indicator_colors.Off[1], fault_indicator_colors.Off[2]));

  if (Indicator_Flags.Miscellaneous) {
    pixels.setPixelColor(Indicator_Indexes.Miscellaneous_Indicator_Index, pixels.Color(fault_indicator_colors.Purple[0], fault_indicator_colors.Purple[1], fault_indicator_colors.Purple[2]));
  }
  else pixels.setPixelColor(Indicator_Indexes.Miscellaneous_Indicator_Index, pixels.Color(fault_indicator_colors.Off[0], fault_indicator_colors.Off[1], fault_indicator_colors.Off[2]));

  pixels.show();
}

void UpdateBatteryHealth(uint8_t bat) {
  Serial.print("Bat SOC");
  Serial.print(bat);
  
  if(bat > 100){
    bat = 0; //this is to ensure that bad data doesnt cause the battery to go higher
  }
  uint8_t scaled_percent = ceil(0.09 * bat);
  // Highlight the red ones:
  uint8_t actual_num = 13-scaled_percent;
  for (int i=5; i<=actual_num; i++) {
    pixels.setPixelColor(i, pixels.Color(100, 0, 0));
  }
  // highlight the green ones
  for (int i=13; i > actual_num; i--) {
    // Serial.print(i);
    pixels.setPixelColor(i, pixels.Color(SOC_Red_Colors[scaled_percent], SOC_Green_Colors[scaled_percent], 0));
  }
  pixels.show();
  // delay(2000);
}

float process_brake_pressure(uint32_t adc) {
  
  float bp_voltage;
  float bp;

  bp_voltage = (adc*vref)/ADC_MAX_VAL ;
  bp = ((bp_voltage - 0.5) / (vref-1)) * 100;


  if (bp > 3) {
    Indicator_Flags.Brake_Status = true;
  }
  else {
    Indicator_Flags.Brake_Status = false;
  }

  return bp;
}

uint16_t process_wheel_speed(uint32_t adc) {

  uint16_t ws_voltage;
  uint16_t ws_frequency;

  ws_voltage = (vref/ADC_MAX_VAL)*adc;

  // VOUT = VCC × f × C1 × R1
  // C1 = 10 nf
  // R1 = LINPOT VALUE

  ws_frequency = ws_voltage / (.00000001 * 5 * 100);

  return ws_frequency;
}

int8_t process_steering_angle(uint32_t adc) {

  uint32_t sa_voltage;

  sa_voltage = (vref/ADC_MAX_VAL)*adc;

  int8_t sa_angle;

  // sa_angle = sa_voltage - 2;
  // Serial.print("SA voltage: ");
  // Serial.println(sa_voltage);

  // Serial.print("SA ADC");
  // Serial.println(adc);

  return sa_voltage;
}

void PlayRTDBuzzer(uint8_t pin_num) {
    digitalWrite(pin_num, HIGH);

    delay(1000); // Delay for 1000 milliseconds (1 second)

    digitalWrite(pin_num, LOW);
    delay(1000);
}