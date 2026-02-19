void calcularCoeficientesPosPID(){
  pq0 = pKp + pKi*Ts/2 + pKd/Ts;
  pq1 = -pKp + pKi*Ts/2 - 2*pKd/Ts;
  pq2 = pKd/Ts;
}

void tustinPosPID(float pos,float saturacion){
  if(movimiento && (vsetPoint == 0)){
    controlPos = true;
    movimiento = false;
    odrv1.setAbsolutePosition(0);
    odrv3.setAbsolutePosition(0);    
  }
  if(controlPos && (vsetPoint != 0)){
    controlPos = false;
    movimiento = true;
    pOutput[0] = 0;
    pOutput[1] = 0;
    pError[0] = 0;
    pError[1] = 0;
    pError[2] = 0;
  }

  if(controlPos){
    pError[0] = 0 - pos;

    pOutput[0] = pOutput[1] + pq0*pError[0] + pq1*pError[1] + pq2*pError[2];
    pOutput[0] = constrain(pOutput[0], -saturacion, saturacion);
    pOutput[1] = pOutput[0];
    pError[2] = pError[1];
    pError[1] = pError[0];
  }
}

void calcularCoeficientesPI(){
  vq0 = vKp + vKi*Ts/2;
  vq1 = -vKp + vKi*Ts/2;
}

void tustinPI(float vel, float saturacion){
  vError[0] = vsetPoint + pOutput[0] - vel;

  vOutput[0] = vOutput[1] + vq0*vError[0] + vq1*vError[1];
  vOutput[0] = constrain(vOutput[0], -saturacion, saturacion); // +10 -10 grados = 0.174 rad
  // Actualizar valores
  vOutput[1] = vOutput[0];
  vError[1] = vError[0];
}

void calcularCoeficientesPID(){
  q0 = Kp + Ki*Ts/2 + Kd/Ts;
  q1 = -Kp + Ki*Ts/2 - 2*Kd/Ts;
  q2 = Kd/Ts;
}

void tustinPID(float vp, float saturacion, float limiteFuncionamiento){
  error[0] = (setPoint + vOutput[0]) - vp;

  output[0] = output[1] + -(q0*error[0] + q1*error[1] + q2*error[2]);
  output[0] = constrain(output[0], -saturacion, saturacion);
  // static int ciclos = 0;
  // if(ciclos<1000){ // ignorar los primeros 1000 ciclos
  //   ciclos++;
  //   output[0] = 0;
  // }
  // Desactivar la salida si vp(ángulo) está fuera del límite de funcionamiento
  if (vp >= limiteFuncionamiento + setPoint){
    arranque = false;
    output[0] = 0;
    pOutput[0] = 0;
    vLsetPoint = 0.0f;
    vRsetPoint = 0.0f;
  } else if (vp <= setPoint - limiteFuncionamiento){
    arranque = false;
    output[0] = 0;
    pOutput[0] = 0;
    vLsetPoint = 0.0f;
    vRsetPoint = 0.0f;
  }
  // Actualizar valores
  output[1] = output[0];
  error[2] = error[1];
  error[1] = error[0];
}

void serialReadBluetooth(){
  while (Serial1.available()){
    char command = Serial1.read();
    Serial1.println(command); // Hacer echo
    switch(command) {
      case 'F':   // Move forward
        noInterrupts();
        vLsetPoint = selectedVel;
        vRsetPoint = selectedVel;
        interrupts();
        break;

      case 'B':   // Move backward
        noInterrupts();
        vLsetPoint = -selectedVel;
        vRsetPoint = -selectedVel;
        interrupts();
        break;

      case 'R':   // Turn right
        noInterrupts();
        vLsetPoint = 1;
        vRsetPoint = -1;
        interrupts();
        break;

      case 'L':   // Turn left
        noInterrupts();
        vLsetPoint = -1;
        vRsetPoint = 1;
        interrupts();
        break;

      case 'G':   // Forward left
        noInterrupts();
        vLsetPoint = 1.5;
        vRsetPoint = 3;
        interrupts();
        break;

      case 'H':   // Forward right
        noInterrupts();
        vLsetPoint = 3;
        vRsetPoint = 1.5;
        interrupts();
        break;

      case 'I':   // Backward left
        noInterrupts();
        vLsetPoint = -1.5;
        vRsetPoint = -3;
        interrupts();
        break;

      case 'J':   // Backward right
        noInterrupts();
        vLsetPoint = -3;
        vRsetPoint = -1.5;
        interrupts();
        break;

      case 'S':   // Stop
        noInterrupts();
        vLsetPoint = 0;
        vRsetPoint = 0;
        interrupts();
        break;

      case 'Y':   // Stand UP
        //arranque
          if (arranque){
            break;
          }
          resetVariables();
          odrv1.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_VEL_RAMP);
          odrv3.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_VEL_RAMP);
          odrv1.setVelocity(-10);
          odrv3.setVelocity(10);
          delay(250);
          odrv1.setVelocity(0);
          odrv3.setVelocity(0);
          delay(250);
          odrv1.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
          odrv3.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
          odrv1.setVelocity(13.5);
          odrv3.setVelocity(-13.5);
          while(1){
            noInterrupts();
            if(pitch >= 0.174){
              break;
            }
            interrupts();
          } // 10°
          interrupts();
          odrv1.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_VEL_RAMP);
          odrv3.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_VEL_RAMP);
          odrv1.setVelocity(0);
          odrv3.setVelocity(0);
          while(1){
            noInterrupts();
            if(pitch >= setPoint){
              break;
            }
            interrupts();
          }
          interrupts();
          odrv1.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
          odrv3.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
          odrv1.setVelocity(0);
          odrv3.setVelocity(0);
          noInterrupts();
          arranque = true;
          interrupts();
        break;

      case 'X':   // Modo cambio de altura
        selectedVel = 1;
        break;

      case 'x':   // Modo cambio de velocidad
        selectedVel = 2;
        break;

      // Set altura o velocidad using single characters (0-9)
      case '0':
        noInterrupts();
        if(escogerAltura){
          altura = 0.05;
        }  
        interrupts(); 
        break;
      case '1': 
        noInterrupts();
        if(escogerAltura){
          altura = 0.13;
        }    
        interrupts(); 
        break;  
      case '2': 
        noInterrupts();
        if(escogerAltura){
          altura = 0.21;
        }   
        interrupts(); 
        break;
      case '3': 
        noInterrupts();
        if(escogerAltura){
          altura = 0.28;
        }   
        interrupts(); 
        break;
      case '4': 
        noInterrupts();
        if(escogerAltura){
          altura = 0.36;
        }     
        interrupts(); 
        break;
      case '5': 
        noInterrupts();
        if(escogerAltura){
          altura = 0.44;
        }    
        interrupts(); 
        break;
      case '6': 
        noInterrupts();
        if(escogerAltura){
          altura = 0.52;
        }   
        interrupts(); 
        break;
      case '7': 
        noInterrupts();
        if(escogerAltura){
          altura = 0.59;
        }  
        interrupts(); 
        break;
      case '8': 
        noInterrupts();
        if(escogerAltura){
          altura = 0.67;
        }
        interrupts();
        break;
      case '9': 
        noInterrupts();
        if(escogerAltura){
          altura = 0.75;
        }
        interrupts();
        break;
    }
  }
}
void serialRead() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();

    if (inChar == '\n' || inChar == '\r') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
  if (stringComplete) {
    if (inputString.length() > 1) {
      currentCommand = inputString.charAt(0);         // Primera letra
      String numberPart = inputString.substring(1);   // El resto
      value = numberPart.toFloat();                   // Convertir a float
      // Identificar comando
      switch (currentCommand) {
        case 'H':
          if((value <= 0.75) && (value >= 0.05)){
            noInterrupts();
            altura = roundToDecimals(value,2);
            interrupts();
          }
          break;
        case 'A':
          //arranque
            if (arranque){
              break;
            }
            resetVariables();
            odrv1.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_VEL_RAMP);
            odrv3.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_VEL_RAMP);
            odrv1.setVelocity(-10);
            odrv3.setVelocity(10);
            delay(250);
            odrv1.setVelocity(0);
            odrv3.setVelocity(0);
            delay(250);
            odrv1.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
            odrv3.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
            odrv1.setVelocity(13.5);
            odrv3.setVelocity(-13.5);
            while(1){
              noInterrupts();
              if(pitch >= 0.174){
                break;
              }
              interrupts();
            } // 10°
            interrupts();
            odrv1.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_VEL_RAMP);
            odrv3.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_VEL_RAMP);
            odrv1.setVelocity(0);
            odrv3.setVelocity(0);
            while(1){
              noInterrupts();
              if(pitch >= setPoint){
                break;
              }
              interrupts();
            }
            interrupts();
            odrv1.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
            odrv3.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
            odrv1.setVelocity(0);
            odrv3.setVelocity(0);
            noInterrupts();
            arranque = true;
            interrupts();
          break;
        case 'P':
          // pKp = value;
          break;
        case 'V':
          value = constrain(value,-5,5);
          noInterrupts();
          vLsetPoint = value;
          vRsetPoint = value;
          interrupts();
          break;
        case 'R':
          value = constrain(value,-1,1);
          noInterrupts();
          vLsetPoint = value/2;
          vRsetPoint = -value/2;
          interrupts();
          break;
        default:
          Serial.println("Comando desconocido.");
          break;
      }
      noInterrupts();
      calcularCoeficientesPID();
      calcularCoeficientesPI();
      calcularCoeficientesPosPID();
      interrupts();
    } else {
      Serial.println("Entrada inválida.");
    }
    // Reset
    inputString = "";
    stringComplete = false;
  }
}
