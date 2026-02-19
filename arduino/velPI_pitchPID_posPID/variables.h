volatile int bluetoothFlag = 0;
volatile float spAux, pitchAux;
volatile bool arranque = false;
volatile float altura = 0.05;
float alturaAux = 0.05;
float selectedVel = 2;
bool escogerAltura = true

;
// Controladores
  float Ts = 0.01; // en segundos

  // Pitch PID
    float Kp = 80.0111; //80 110
    float Ki = 190.0111; //180 190
    float Kd = 0.50111f; //0.5 06
    float q0, q1, q2; // coeficientes para control incremental
    float setPoint = 0.251f; //16.73° 0.292 11° 0.192  0.251
    float error[3] = {0.0f, 0.0f, 0.0f};
    float output[2] = {0.0f, 0.0f};

  // Velocidad PI
    float vKp = 0.0080111f; // 0.008
    float vKi = 0.015011f; // 0.015
    float vq0, vq1; // coeficientes para control incremental
    volatile float vLsetPoint = 0.0f;
    volatile float vRsetPoint = 0.0f;
    float vsetPoint = 0.0f;
    float vError[2] = {0.0f, 0.0f};
    float vOutput[2] = {0.0f, 0.0f};

  // Posicion PID
    float pKp = 0.0f; //1.1
    float pKi = 0.0f; //0.01
    float pKd = 0.0f; //0.8
    float pq0, pq1, pq2;
    float pError[3] = {0.0f, 0.0f, 0.0f};
    float pOutput[2] = {0.0f, 0.0f};
    bool movimiento = true;
    bool controlPos = false;

// IMU
  float gx, gy, gz, ax, ay, az;
  float pitch, roll;
  float deltat;

// Feedback Odrive
  volatile float vLeft = 0.0f;
  volatile float vRight = 0.0f;
  volatile float posLeft = 0.0f;
  volatile float posRight = 0.0f;
  float vAverage = 0.0f;
  float posAverage = 0.0f;

// Comandos Serial
  String inputString = "";     // Almacena el string recibido
  boolean stringComplete = false;
  char currentCommand = '\0'; // O, A, P, I o D
  float value = 0.0;

void resetVariables(){
  error[0] = 0;
  error[1] = 0;
  error[2] = 0;
  pError[0] = 0;
  pError[1] = 0;
  pError[2] = 0;
  vError[0] = 0;
  vError[1] = 0;
  output[0] = 0;
  output[1] = 0;
  pOutput[0] = 0;
  pOutput[1] = 0;
  vOutput[0] = 0;
  vOutput[1] = 0;
  movimiento = true;
  controlPos = false;
}