
float altura = 0.05;
unsigned long prevTime;
unsigned long tiempoEjec = 0;

// Parámetros control roll Fuzzy
float fuzzyError = 0.0f;
float fuzzyOutput = 0;

// Parámetros control roll PID
float hKp = 0.3f; // 0.1
float hKi = 1.4f; // 1.8
float hKd = 0.005f; // 0.01
float hq0, hq1, hq2; // coeficientes para control incremental
float hsetPoint = 0.0f;
float herror[3] = {0.0f, 0.0f, 0.0f};
float houtput[2] = {0.0f, 0.0f};
float hLoutput = 0.0f;
float hRoutput = 0.0f;
float Ts = 0.01; // en segundos

// IMU
float gx, gy, gz, ax, ay, az;
float pitch, roll;
float deltat;

// Comandos serial
String inputString = "";     // Almacena el string recibido
boolean stringComplete = false;
char currentCommand = '\0'; // O, A, P, I o D
float value = 0.0;
bool on = false;