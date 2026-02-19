#include "variables.h"
#include "nodo_imu.h"
#include "nodo_odrive.h"
#include "nodo_PID.h"

IntervalTimer myTimer;

void isr_PID(){
  
  leerIMU();
  if (arranque){
    vsetPoint = (vLsetPoint + vRsetPoint)/2;
    vAverage = (vLeft + vRight)/2;
    posAverage = (posLeft + posRight)/2;
    /* Lectura de */
    //Control Pos
    //tustinPosPID(posAverage, 5);
    //Control Vel
    tustinPI(vAverage, 0.28);
    // Control Pitch
    tustinPID(pitch, 12.5, 0.56);

    odrv1.setVelocity(-(output[0] + vLsetPoint + pOutput[0]));
    odrv3.setVelocity(output[0] + vRsetPoint + pOutput[0]);
  }

  if (altura > alturaAux){
    alturaAux += 0.002;
    odrv0.setPosition(-alturaAux);
    odrv2.setPosition(alturaAux);
  } else if (altura < alturaAux){
    alturaAux -= 0.002;
    odrv0.setPosition(-alturaAux);
    odrv2.setPosition(alturaAux);
  }
  // ********VALORES TESIS BRUNO*************

  // spAux = bfs::rad2deg(setPoint + vOutput[0]);
  // pitchAux = bfs::rad2deg(pitch);
  // bluetoothFlag++;

  // ********MUCHOS VALORES************
  // Serial.print(Kp,2);
  // Serial.print("; ");
  // Serial.print(Ki,2);
  // Serial.print("; ");
  // Serial.print(Kd,2);
  // Serial.print("; ");
  // Serial.print("Setpoint:");
  // Serial.print(bfs::rad2deg(setPoint + vOutput[0]));
  // Serial.print(",Pitch:");
  // Serial.print(bfs::rad2deg(pitch));
  // Serial.print(",vel:");
  // Serial.println(vAverage);
  // Serial.print("; ");
  // Serial.print(output[0]);
  // Serial.print("; ");
  // Serial.print(vLeft);
  // Serial.print("; ");
  // Serial.print(vRight);
  // Serial.print("; ");
  // Serial.println(vAverage);
  // Serial.print("posRight:");
  // Serial.print(posRight);
  // Serial.print(",");
  // Serial.print("posLeft:");
  // Serial.println(posLeft);
}

void setup() {
  inputString.reserve(20);
  /* Inicia Serial */
  Serial1.begin(9600);
  Serial.begin(115200);
  // while(!Serial) {}
  iniciarIMU();

  calcularCoeficientesPID();
  calcularCoeficientesPI();
  calcularCoeficientesPosPID();

  iniciarOdrive();
  static int calibIMU = 0;
  while(calibIMU <= 1000){
    leerIMU();
    delay(10);
    calibIMU++;
  }
  enablePosControl(odrv0, "ODrive0");
  enablePosControl(odrv2, "ODrive2");
  myTimer.begin(isr_PID, Ts*1000000);
}

void loop() {
  serialReadBluetooth();
  // if (bluetoothFlag > 1) {
  //   bluetoothFlag = 0;

  //   // ********VALORES TESIS CHRISTIAN*************
  //   Serial1.print("S ");
  //   Serial1.print(spAux);
  //   Serial1.print(" P ");
  //   Serial1.println(pitchAux);
  // }
  pumpEvents(can_intf);
  if (odrv1_user_data.received_feedback || odrv3_user_data.received_feedback) {
    Get_Encoder_Estimates_msg_t feedback1 = odrv1_user_data.last_feedback;
    odrv1_user_data.received_feedback = false;
    Get_Encoder_Estimates_msg_t feedback3 = odrv3_user_data.last_feedback;
    odrv3_user_data.received_feedback = false;
    noInterrupts();
    vLeft = roundToDecimals(-feedback1.Vel_Estimate, 2);
    vRight = roundToDecimals(feedback3.Vel_Estimate, 2);
    posLeft = roundToDecimals(-feedback1.Pos_Estimate, 2);
    posRight = roundToDecimals(feedback3.Pos_Estimate, 2);
    interrupts(); 
  }
}
