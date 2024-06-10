#include <stdio.h>
#include <math.h>
#include <string.h>

#define DRIVE_STATE_ID 0x2f0a000
#define DRIVE_STATE_MSG_INDEX 10
#define DRIVE_STATE_MSG_LENGTH 4

#define VCU_STATES_ID 0x2f0a002
#define LAUNCH_MODE_MSG_INDEX 3
#define LAUNCH_MODE_MSG_MASK 0b00000100
#define LAUNCH_MODE_MSG_SHIFT 2
#define TRANS_BRAKE_MSG_INDEX 4
#define TRANS_BRAKE_MSG_MASK 0b00000010
#define TRANS_BRAKE_MSG_SHIFT 1
// #define TRANS_BRAKE_SWITCH_MSG_INDEX 3
// #define TRANS_BRAKE_SWITCH_MSG_MASK 0b01000000
// #define TRANS_BRAKE_SWITCH_MSG_SHIFT 6
#define COOLING_STATUS_MSG_INDEX 4
#define COOLING_STATUS_MSG_MASK 0b10000000
#define COOLING_STATUS_MSG_SHIFT 7

#define VCU_FAULTS_ID 0x2f0a044
#define APP1_LO_MSG_INDEX 0
#define APP1_LO_MSG_MASK 0b00010000
#define APP1_LO_MSG_SHIFT 4
#define APP1_HI_MSG_INDEX 0
#define APP1_HI_MSG_MASK 0b00100000
#define APP1_HI_MSG_SHIFT 5
#define APP2_LO_MSG_INDEX 0
#define APP2_LO_MSG_MASK 0b00000010
#define APP2_L0_MSG_SHIFT 1
#define APP2_HI_MSG_INDEX 0
#define APP2_HI_MSG_MASK 0b00000100
#define APP2_HI_MSG_SHIFT 2
#define APP_CHECK_MSG_INDEX 1
#define APP_CHECK_MSG_MASK 0b10000000
#define APP_CHECK_MSG_SHIFT 7

#define DASH_MSG_ID 0x234

#define AMS_MSG_ID 0x100

typedef struct {
  uint8_t rlws_LSB;
  uint8_t rlws_MSB;
  uint8_t flws_LSB;
  uint8_t flws_MSB;
  uint8_t sa_LSB;
  uint8_t sa_MSB;
  uint8_t faults;
} Can_Send_Bytes;

// Handle Faults Like this: (BSPD_STATUS | (misc_button << 1) & 0b00000011)
Can_Send_Bytes Updated_Dash_Data = {
  0,
  0,
  0,
  0,
  0,
  0,
  0
};

typedef struct {
  uint8_t battery_soc_percent;
  int8_t max_temp;
  int8_t min_temp;
  float_t actual_pack_voltage;
} Can_Receive_Bytes;

Can_Receive_Bytes Updated_Rx_Data = {
  0,
  0,
  0,
  0
};


