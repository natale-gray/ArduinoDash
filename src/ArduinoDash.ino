#include "can_helpers.hpp"
#include <Arduino_CAN.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <Adafruit_NeoPixel.h>
#include <helper_functions.hpp>

// uint16_t const max_voltage = 3984; // 3984 = 398.4V
// uint16_t const max_current = 50; // 50 = 5.0A
// uint8_t enable_data[] = {(max_voltage >> 8) & 0xFF, max_voltage & 0xFF, (max_current >> 8) & 0xFF, max_current & 0xFF,0x0};
// uint8_t disable_data[] = {(max_voltage >> 8) & 0xFF, max_voltage & 0xFF, (max_current >> 8) & 0xFF, max_current & 0xFF,0x1};
// CanMsg enable_msg = CanMsg(CanExtendedId(0x1806E5F4), 5, enable_data);
// CanMsg disable_msg = CanMsg(CanExtendedId(0x1806E5F4), 5, disable_data);
// uint16_t count = 0;


#define BAUDRATE 1000000

//define initial states 
uint32_t sa_voltage;
uint32_t sa_sensor_angle;
uint32_t sa_wheel_angle;

uint32_t rearleftws;
uint32_t frontleftws;
uint32_t inertia;

uint32_t drive;

float brake_pressure;
uint32_t coolant_temp;

uint8_t battery_health = 3;

uint8_t is_bytes[8] {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, inertia ? 0x01 : 0x00};
// char rtd_bytes[8] {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, drive ? 0xFF : 0x00};
// char flws_bytes[8] {0xFF, 0xFF, 0xFF, 0xFF, 
//               ((frontleftws >> 24) & 0xFF), ((frontleftws >> 16) & 0xFF), 
//               ((frontleftws >> 8) & 0xFF), (frontleftws & 0xFF)};
// char sa_bytes[8] {0xFF, 0xFF, 0xFF, 0xFF, 
//               ((sa_wheel_angle >> 24) & 0xFF), ((sa_wheel_angle >> 16) & 0xFF), 
//               ((sa_wheel_angle >> 8) & 0xFF), (sa_wheel_angle & 0xFF)};
// char rlws_bytes[8] {0xFF, 0xFF, 0xFF, 0xFF, 
//               ((rearleftws >> 24) & 0xFF), ((rearleftws >> 16) & 0xFF), 
//               ((rearleftws >> 8) & 0xFF), (rearleftws & 0xFF)};

void setup() {

  analogReadResolution(14);
  pinMode(SA_PIN, INPUT_PULLUP);
  // pinMode(FLWS, INPUT);
  // pinMode(RLWS, INPUT);
  pinMode(IS_PIN, INPUT);
  // pinMode(LED_PIN, INPUT);
  // pinMode(CANRX, INPUT);
  pinMode(DRIVESIGNAL_PIN, INPUT);
  // pinMode(CANTX, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(9600);
  Serial1.begin(BAUDRATE, SERIAL_8N1);
  Serial1.setTimeout(1000);
  pixels.clear(); // Set all pixel colors to 'off'

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)



  if (!CAN.begin(CanBitRate::BR_500k)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  Serial.print("Hello this the the Dashboard Code\r\n");
}


void loop() {
  delay(500);


  //check to see if a signal is recieved from one of the inputs
  brake_pressure = 0.5 + analogRead(BP_PIN) * 4 / 1024;
  Serial.print("bp: ");
  Serial.println(brake_pressure);
  if (brake_pressure < 0.5) Indicator_Flags.Brake_Status = true;

  rearleftws = analogRead(RLWS_PIN);
  frontleftws = analogRead(FLWS_PIN);
  Serial.print("rlws: ");
  Serial.println(rearleftws);



  if(digitalRead(IS_PIN) == HIGH) {
    inertia = true;
  }
  else{
    inertia = false;
  }
  if(digitalRead(DRIVESIGNAL_PIN) == HIGH) {
    drive = true;
  }
  else{
    drive = false;
  }
  // ReadSA();
  // PlayRTDBuzzer(BUZZER_PIN);

  UpdateBatteryHealth(battery_health);
  UpdateIndicators();

  // prepare_can_data();

  // send_can_data();

}


void ReadSA(void) {
  sa_voltage = analogRead(SA_PIN);
  sa_sensor_angle = ((sa_voltage - 0.5) / 4) * (MAX_RIGHT_SA_SENSOR - MAX_LEFT_SA_SENSOR) + MAX_LEFT_SA_SENSOR;
  sa_wheel_angle = ((sa_sensor_angle - MAX_LEFT_SA_SENSOR) / (MAX_RIGHT_SA_SENSOR - MAX_LEFT_SA_SENSOR)) * (MAX_RIGHT_SA_WHEEL - MAX_LEFT_SA_WHEEL) + MAX_LEFT_SA_WHEEL;
  
  Serial.print("sa_voltage = "), Serial.print(sa_voltage), Serial.print('\n');
  Serial.print("sa_sensor_angle = "), Serial.print(sa_sensor_angle), Serial.print('\n');
  Serial.print("sa_wheel_angle = "), Serial.print(sa_wheel_angle), Serial.print('\n');
}


// void prepare_can_data(void){

//   // uint8_t const msg_data[] = {0xCA,0xFE,0,0,0,0,0,0};
//   // memcpy((void *)(msg_data + 4), &msg_cnt, sizeof(msg_cnt));

//   // CanMsg msg(CAN_ID, sizeof(msg_data), msg_data);

//   is_bytes[7] = inertia ? 0xFF : 0x00;
//   rtd_bytes[7] = drive ? 0xFF : 0x00;

//   sa_bytes[7] = ((sa_wheel_angle >> 24) & 0xFF);
//   sa_bytes[6] = ((sa_wheel_angle >> 16) & 0xFF);
//   sa_bytes[5] = ((sa_wheel_angle >> 8) & 0xFF);
//   sa_bytes[4] = (sa_wheel_angle & 0xFF);

//   flws_bytes[7] = ((frontleftws >> 24) & 0xFF);
//   flws_bytes[6] = ((frontleftws >> 16) & 0xFF);
//   flws_bytes[5] = ((frontleftws >> 8) & 0xFF);
//   flws_bytes[4] = (frontleftws & 0xFF);

//   rlws_bytes[7] = ((rearleftws >> 24) & 0xFF);
//   rlws_bytes[6] = ((rearleftws >> 16) & 0xFF);
//   rlws_bytes[5] = ((rearleftws >> 8) & 0xFF);
//   rlws_bytes[4] = (rearleftws & 0xFF);

//   memcpy(msg_data[IS_STATUS - START_ITEM], is_bytes, 8);
//   memcpy(msg_data[STEERING_ANGLE - START_ITEM], sa_bytes, 8);
//   memcpy(msg_data[RTD_STATUS - START_ITEM], rtd_bytes, 8);
//   memcpy(msg_data[RLWS - START_ITEM], rlws_bytes, 8);
//   memcpy(msg_data[FLWS - START_ITEM], flws_bytes, 8);

// }

uint16_t count = 0;

void send_can_data(void){

  CanMsg is_message = CanMsg(CanExtendedId(0x1806E5F4), 8, is_bytes);

  if(count > 1){
    int ret = CAN.write(is_message);
    // Serial.println(enable_msg);
    if(!(ret == 0 || ret == 1)){
        Serial.print("CAN Error: ");
        Serial.println(ret);
        CAN.clearError();
    }
    count = 0;
  }

  if(CAN.available()){
    
    CanMsg msg = CAN.read();
    Serial.print("Received: ");
    float current_voltage = ((msg.data[0] << 8) | msg.data[1]) / 10.0;
    float current_amps = ((msg.data[2] << 8) | msg.data[3]) / 10.0;
    Serial.print("Voltage: ");
    Serial.print(current_voltage);
    Serial.print("V, Current: ");
    Serial.print(current_amps);
    Serial.println("A");
    uint8_t status = msg.data[4];
    Serial.print("Status: ");
    Serial.println(status & 0x1); //Hardware Fault
    Serial.println(status >> 0x1 & 0x1); // Charger OverTemp
    Serial.println(status >> 0x2 & 0x1); // Input voltage fault
    Serial.println(status >> 0x3 & 0x1); // Battery connection fault
    Serial.println(status >> 0x4 & 0x1); // communication fault

  }

  count++;
  delay(10);

  // for(uint32_t addr = START_ITEM; addr < LAST_ITEM; addr++){
    
  //   Serial.println(addr);
  //   Serial.println(addr - START_ITEM);
  //   Serial.println(sizeof(msg_data[addr - START_ITEM]));
  //   for(size_t i = 0; i < sizeof(msg_data[addr - START_ITEM]); i++) {
  //       Serial.print(msg_data[addr - START_ITEM][i], DEC); // Print byte in decimal format
  //       Serial.print(" "); // Print space between bytes
  //   }
  //   Serial.println();
  //   // CAN.beginPacket(0x12);

  //   int ret = CAN.write(CanMsg(CanStandardId(addr), sizeof(msg_data[addr - START_ITEM]), msg_data[addr - START_ITEM]));
  //   if(ret != 0){
  //     Serial.println("Failed to send CAN message");

  //     CAN.clearError();
  //   }
  //   // Serial.println(ret);
  // }
}