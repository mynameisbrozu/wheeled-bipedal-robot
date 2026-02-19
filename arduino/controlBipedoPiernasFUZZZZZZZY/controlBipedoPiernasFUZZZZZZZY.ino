#include "variables.h"
#include "nodo_odrive.h"
#include "nodo_imu.h"
#include "nodo_PID.h"
#include "nodo_fuzzy.h"

IntervalTimer myTimer;

void isr_PID(){
  /* Lectura de */
  leerIMU();
  //tustinPID(roll, altura);
  prevTime = micros();
  controlFuzzy(roll, altura);
  tiempoEjec = micros() - prevTime;

  odrv0.setPosition(-(altura - hLoutput));
  odrv2.setPosition(altura + hRoutput);

  // Serial.print(hKp,3);
  // Serial.print("; ");
  // Serial.print(hKi,3);
  // Serial.print("; ");
  // Serial.print(hKd,3);
  // Serial.print("; ");  
  // Serial.print(bfs::rad2deg(roll));
  // Serial.print("; ");
  // Serial.print("R:");
  // Serial.print(altura + hRoutput);
  // Serial.print("; ");
  // Serial.print(",L:");
  // Serial.print((altura - hLoutput));

  Serial.print(",Setpoint:");
  Serial.print(bfs::rad2deg(hsetPoint));
  Serial.print(",Roll:");  
  Serial.print(bfs::rad2deg(roll));
  Serial.print(",Tiempo:");  
  Serial.println(tiempoEjec);
  // Serial.print(",Output:");  
  // Serial.println(fuzzyOutput, 6);

  // Serial.print("odrv0-pos:");
  // Serial.print(odrv0_pos);
  // Serial.print(",");
  // Serial.print("odrv2-pos:");
  // Serial.println(odrv2_pos);
}

void setup() {
  inputString.reserve(20);
  /* Inicia Serial */
  Serial.begin(115200);
  while(!Serial) {}
  iniciarIMU();

  //calcularCoeficientesPID();
  //calcularCoeficientesPID();
  crearFuzzy();

  iniciarOdrive();

  myTimer.begin(isr_PID, Ts*1000000);
}

void loop() {
  serialRead();
  // pumpEvents(can_intf);
  // if (odrv0_user_data.received_feedback) {
  //   Get_Encoder_Estimates_msg_t feedback0 = odrv0_user_data.last_feedback;
  //   odrv0_user_data.received_feedback = false;
  //   noInterrupts();
  //   odrv0_pos = feedback0.Pos_Estimate;
  //   interrupts();
  // }
  // if (odrv2_user_data.received_feedback) {
  //   Get_Encoder_Estimates_msg_t feedback2 = odrv2_user_data.last_feedback;
  //   odrv2_user_data.received_feedback = false;
  //   noInterrupts();
  //   odrv2_pos = feedback2.Pos_Estimate;
  //   interrupts();
  // }    
}
