void calcularCoeficientesPID(){
  hq0 = hKp + hKi*Ts/2 + hKd/Ts;
  hq1 = -hKp + hKi*Ts/2 - 2*hKd/Ts;
  hq2 = hKd/Ts;
}

void tustinPID(float vp, float altura){
  herror[0] = hsetPoint - vp;

  houtput[0] = houtput[1] + -(hq0*herror[0] + hq1*herror[1] + hq2*herror[2]);
  //houtput[0] = constrain(houtput[0], 0.05-altura, 0.75 - altura);
  static int ciclos3 = 0;
  if(ciclos3<1000){ // ignorar los primeros 1000 ciclos
    ciclos3++;
    houtput[0] = 0;
  }

  hRoutput = houtput[0];
  hLoutput = houtput[0];

  if (altura + houtput[0] > 0.75) {
    hRoutput = 0.75 - altura;    
  } else if (altura + houtput[0] < 0.05) {
    hRoutput = 0.05 - altura;    
  }

  if (altura - houtput[0] > 0.75) {
    hLoutput = -(0.75 - altura);    
  } else if (altura - houtput[0] < 0.05) {
    hLoutput = -(0.05 - altura);    
  }

  houtput[1] = houtput[0];
  herror[2] = herror[1];
  herror[1] = herror[0];
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
          if(value <= 0.75 && value >= 0.05){
            // noInterrupts();
            // odrv0.setPosition(-value);
            // odrv2.setPosition(value);
            // interrupts();
          }
          break;
        case 'A':
          noInterrupts();
          hsetPoint = roundToDecimals(bfs::deg2rad(value),3);
          interrupts();
          break;
        case 'P':
          hKp = value;
          break;
        case 'I':
          hKi = value;
          break;
        case 'D':
          hKd = value;
          break;
        default:
          Serial.println("Comando desconocido.");
          break;
      }
      noInterrupts();
      calcularCoeficientesPID();
      interrupts();
    } else {
      Serial.println("Entrada inválida.");
    }
    // Reset
    inputString = "";
    stringComplete = false;
  }
}
