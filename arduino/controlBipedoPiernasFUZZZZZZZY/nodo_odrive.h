#include "ODriveCAN.h"
// CAN bus baudrate
#define CAN_BAUDRATE 1000000

// ID del nodo de los odrives
#define ODRV0_NODE_ID 0
#define ODRV1_NODE_ID 1
#define ODRV2_NODE_ID 2
#define ODRV3_NODE_ID 3

#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;   

// Instantiate ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID);
ODriveCAN odrv2(wrap_can_intf(can_intf), ODRV2_NODE_ID);
ODriveCAN odrv3(wrap_can_intf(can_intf), ODRV3_NODE_ID);
ODriveCAN* odrives[] = {&odrv0, &odrv1, &odrv2, &odrv3};

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Keep some application-specific user data for every ODrive.
ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;
ODriveUserData odrv2_user_data;
ODriveUserData odrv3_user_data;

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }
}

bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

void linkOdrive(ODriveUserData &odrv_user_data, const char* odrv_name) {
  while (!odrv_user_data.received_heartbeat) {
    pumpEvents(can_intf);
    delay(100);
  }
  Serial.print(odrv_name); 
  Serial.println(" encontrado");
}

void enableClosedLoopControl(ODriveCAN &odrv, ODriveUserData &odrv_user_data, const char* odrv_name) {
  while (odrv_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv.clearErrors();
    delay(1);
    odrv.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
  }
  Serial.print(odrv_name);
  Serial.println(" activado");
}

void enableTrajectoryControl(ODriveCAN &odrv, const char* odrv_name) {
  odrv.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_TRAP_TRAJ);
  Serial.print(odrv_name);
  Serial.println(" en control de trayectoria");
}

void enablePosControl(ODriveCAN &odrv, const char* odrv_name) {
  odrv.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  Serial.print(odrv_name);
  Serial.println(" en control de posición normal");
}

void iniciarOdrive(){
  // Register callbacks for the heartbeat and encoder feedback messages
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);

  odrv1.onFeedback(onFeedback, &odrv1_user_data);
  odrv1.onStatus(onHeartbeat, &odrv1_user_data);

  odrv2.onFeedback(onFeedback, &odrv2_user_data);
  odrv2.onStatus(onHeartbeat, &odrv2_user_data);

  odrv3.onFeedback(onFeedback, &odrv3_user_data);
  odrv3.onStatus(onHeartbeat, &odrv3_user_data);

  // Configure and initialize the CAN bus interface.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }
  Serial.println("Esperando ODrives...");
  
  linkOdrive(odrv0_user_data, "ODrive0");
  linkOdrive(odrv1_user_data, "ODrive1");
  linkOdrive(odrv2_user_data, "ODrive2");
  linkOdrive(odrv3_user_data, "ODrive3");

  Serial.println("Enabling closed loop control...");

  enableTrajectoryControl(odrv0, "ODrive0");
  enableTrajectoryControl(odrv2, "ODrive2");
  enableClosedLoopControl(odrv0, odrv0_user_data, "ODrive0");
  enableClosedLoopControl(odrv2, odrv2_user_data, "ODrive2");

  odrv0.setPosition(-altura);
  odrv2.setPosition(altura);
  
  delay(3000);

  enablePosControl(odrv0, "ODrive0");
  enablePosControl(odrv2, "ODrive2");
  enablePosControl(odrv0, "ODrive0");
  enablePosControl(odrv2, "ODrive2");
}

