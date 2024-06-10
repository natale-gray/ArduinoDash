#include "can_helpers.hpp"
#include <Arduino_CAN.h>
// #include <experimental/filesystem>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <Adafruit_NeoPixel.h>
#include <helper_functions.hpp>

#define BAUDRATE 1000000

//define initial states 
uint8_t loop_count = 0;

bool drive;
bool last_drive;

// Note frequencies (in Hz) for the "ray ray, UVA" part
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_E5 659

// Note durations (in milliseconds)
#define QUARTER_NOTE 375
#define HALF_NOTE 750

int melody[] = {NOTE_G4, NOTE_G4, NOTE_A4, NOTE_G4, NOTE_E5};
int noteDurations[] = {HALF_NOTE, HALF_NOTE, QUARTER_NOTE, QUARTER_NOTE, QUARTER_NOTE};

void setup() {

  analogReadResolution(14);
  pinMode(SA_PIN, INPUT_PULLUP);
  // pinMode(FLWS_PIN, INPUT);
  // pinMode(RLWS_PIN, INPUT);
  pinMode(IS_PIN, INPUT);
  // pinMode(LED_PIN, INPUT);
  // pinMode(CANRX, INPUT);
  pinMode(DRIVESIGNAL_PIN, INPUT);
  // pinMode(CANTX, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MISC_PIN, INPUT_PULLUP);

  Serial.begin(9600);
  Serial1.begin(BAUDRATE, SERIAL_8N1);
  Serial1.setTimeout(1000);

  pixels.clear(); // Set all pixel colors to 'off'
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)

  CAN.setMailboxMask(4, 0x1FFFFFFF);
  CAN.setMailboxMask(5, 0x1FFFFFFF);

  // CAN.setMailboxMask(6, 0x1FFFFFFF); 

  for (int c=16; c <= 17; c++) {
    CAN.setMailboxID(c, DRIVE_STATE_ID);
  }
  for (int c=18; c <= 19; c++) {
    CAN.setMailboxID(c, VCU_STATES_ID);
  }
  for (int c=20; c<= 21; c++) {
    CAN.setMailboxID(c, VCU_FAULTS_ID);
  }

  if (!CAN.begin(CanBitRate::BR_500k)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
  Serial.print("Hello this the the Dashboard Code\r\n");
}


void loop() {
  if (loop_count == 10) {
    loop_count = 0;
    UpdateBatteryHealth(Updated_Rx_Data.battery_soc_percent);
  }
  loop_count= loop_count + 1;

  process_brake_pressure(analogRead(BP_PIN));

  // Serial.print("Brake Pressure: ");
  // Serial.println(process_brake_pressure(analogRead(BP_PIN)));

  // Serial.println(analogRead(RLWS_PIN));
  // Serial.println(analogRead(RLWS_PIN));
  uint16_t rearleftws= process_wheel_speed(analogRead(RLWS_PIN));
  uint16_t frontleftws = process_wheel_speed(analogRead(FLWS_PIN));
  Updated_Dash_Data.rlws_LSB = (rearleftws & 0xFF00);
  Updated_Dash_Data.rlws_MSB = ((rearleftws >> 4) & 0xFF00);
  Updated_Dash_Data.flws_LSB = (frontleftws & 0xFF00);
  Updated_Dash_Data.flws_MSB = ((frontleftws >> 4) & 0xFF00);
  // Serial.print("RL Wheel Speed: ");
  // Serial.println(rearleftws);
  // Serial.print("FL Wheel Speed: ");
  // Serial.println(frontleftws);

  Indicator_Flags.Miscellaneous = digitalRead(MISC_PIN);

  Serial.print("MISC BUTTON");
  Serial.println(Indicator_Flags.Miscellaneous);

  uint16_t steering_angle =process_steering_angle(analogRead(SA_PIN));
  Updated_Dash_Data.sa_LSB = (steering_angle & 0xFF00);
  Updated_Dash_Data.sa_MSB = ((steering_angle >> 4) & 0xFF00);


  if(drive == true && last_drive == false){
    PlayRTDBuzzer(BUZZER_PIN);
  }
  last_drive = drive;

  UpdateIndicators();

  send_can_data();

  while (CAN.available()) {
    // Serial.println("CAN IS AVAILABLE");
    CanMsg msg = CAN.read();
    handleCanMessage(msg);
  }
  // delay(1000);
}

void send_can_data(void){
  Updated_Dash_Data.faults = ((Indicator_Flags.BSPD_Fault | (Indicator_Flags.Miscellaneous << 1)) & 0b00000011);

  uint8_t bytes[8] = {Updated_Dash_Data.rlws_LSB,
                      Updated_Dash_Data.rlws_MSB,
                      Updated_Dash_Data.flws_LSB,
                      Updated_Dash_Data.flws_MSB,
                      Updated_Dash_Data.sa_LSB,
                      Updated_Dash_Data.sa_MSB,
                      Updated_Dash_Data.faults,
                      0x00
                      };

  CanMsg dash_message = CanMsg(CanExtendedId(DASH_MSG_ID), 8, bytes);

  int ret = CAN.write(dash_message);
  // Serial.println(dash_message);
  if(!(ret == 0 || ret == 1)){
      Serial.print("CAN Error: ");
      Serial.println(ret);
      CAN.clearError();
  }
}

void handleCanMessage(const CanMsg &msg) {
  // Serial.println(msg.id);
  switch (msg.getExtendedId()) {
    case VCU_FAULTS_ID:
      handle_vcu_faults(msg);
      break;
    case VCU_STATES_ID:
      handle_vcu_states(msg);
      break;
    case DRIVE_STATE_ID:
      // handle_drive_state(msg);
    case AMS_MSG_ID:
      handle_ams_data(msg);
    default:
      // Unknown message ID, ignore or handle as needed
      break;
  }
}

void handle_vcu_states(const CanMsg &msg) {
  // Serial.print("Trans Brake Status: ");
  // Serial.println((msg.data[TRANS_BRAKE_MSG_INDEX] & TRANS_BRAKE_MSG_MASK) >> TRANS_BRAKE_MSG_SHIFT);

  // Serial.print("Launch Mode: ");
  // Serial.println((msg.data[LAUNCH_MODE_MSG_INDEX] & LAUNCH_MODE_MSG_MASK) >> LAUNCH_MODE_MSG_SHIFT);
  // if (msg.data[LAUNCH_MODE_MSG_INDEX] == 1) {
  //   Indicator_Flags.Launch_Control = true;
  // }
  // else {
  //   Indicator_Flags.Launch_Control = false;
  // }
  
  // Serial.print("Cooling Status: ");
  // Serial.println((msg.data[COOLING_STATUS_MSG_INDEX] & COOLING_STATUS_MSG_MASK) >> COOLING_STATUS_MSG_SHIFT);
  if (msg.data[COOLING_STATUS_MSG_INDEX] == 1) {
    Indicator_Flags.Cooling_Indicator = true;
  }
  else {
    Indicator_Flags.Cooling_Indicator = false;
  }



}

void handle_vcu_faults(const CanMsg &msg) {

  if (((msg.data[APP1_LO_MSG_INDEX] & APP1_LO_MSG_MASK) >> APP1_LO_MSG_SHIFT)  || ((msg.data[APP1_HI_MSG_INDEX] & APP1_HI_MSG_MASK) >> APP1_HI_MSG_SHIFT) ||
      ((msg.data[APP2_LO_MSG_INDEX] & APP2_LO_MSG_MASK) >> APP2_L0_MSG_SHIFT) || ((msg.data[APP2_HI_MSG_INDEX] & APP2_HI_MSG_MASK) >> APP2_HI_MSG_SHIFT) || ((msg.data[APP_CHECK_MSG_INDEX] & APP_CHECK_MSG_MASK) >> APP_CHECK_MSG_SHIFT)) {
        Indicator_Flags.APPS_Fault = true;
      }
  else {
    Indicator_Flags.APPS_Fault = false;
  }
}

void handle_drive_state(const CanMsg &msg) {
  uint8_t drive_state = msg.data[DRIVE_STATE_MSG_INDEX];
  if (drive_state == 1) {
    drive = true;
  }
  else {
    drive = false;
  }
}

void handle_ams_data(const CanMsg &msg) {
  // Serial.println("AMS_DATA HANDLING");
  Updated_Rx_Data.battery_soc_percent = msg.data[1];
  Indicator_Flags.AMS_Fault = (msg.data[0] >> 1) & 0x1;
  Indicator_Flags.IMD_Fault = (msg.data[0]) & 0x1;
  Updated_Rx_Data.max_temp = msg.data[4];
  Updated_Rx_Data.min_temp = msg.data[5];
  uint16_t scaled_pack_voltage = ((msg.data[2] << 8) | msg.data[3]);
  Updated_Rx_Data.actual_pack_voltage = scaled_pack_voltage / 10.0;
}

void PlayRTDBuzzer(){
  // Play the melody
  for (int i = 0; i < 5; i++) {
    tone(BUZZER_PIN, melody[i], noteDurations[i]);
    delay(noteDurations[i] * 1.30);  // Add a small delay between notes
    noTone(BUZZER_PIN);              // Stop the tone
  }
}