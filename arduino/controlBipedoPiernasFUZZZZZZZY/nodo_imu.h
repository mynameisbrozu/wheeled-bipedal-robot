#include "mpu9250.h"
#include "units.h"
#include "SensorFusion.h"
SF fusion;

/* objeto MPU9250 */
bfs::Mpu9250 imu;

float roundToDecimals(float value, int decimals) {
  float factor = pow(10, decimals);
  return round(value * factor) / factor;
}

void configurarIMU(bool status, const char* msg) {
  if (!status) {
    Serial.println(msg);
    while (1) {}
  }
}


void iniciarIMU(){
  /* Inicia el bus I2C con la dirección 0x68*/
  Wire.begin();
  Wire.setClock(400000);
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
  /* Conexión con la IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  /* Configuraciones */
  configurarIMU(imu.ConfigSrd(5), "Error configurando SRD"); // rate = 1000/(srd+1) Hz
  configurarIMU(imu.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_4G), "Error configurando acelerómetro");
  configurarIMU(imu.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_500DPS), "Error configurando giroscopio"); //1000?
  configurarIMU(imu.ConfigDlpfBandwidth(bfs::Mpu9250::DLPF_BANDWIDTH_92HZ), "Error configurando filtro DLPF"); //20?
}

void leerIMU() {
  if (imu.Read()) {
    ax = imu.accel_x_mps2();
    ay = imu.accel_y_mps2();
    az = imu.accel_z_mps2();
    gx = imu.gyro_x_radps();
    gy = imu.gyro_y_radps();
    gz = imu.gyro_z_radps();
  }
  deltat = fusion.deltatUpdate();
  fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);
  pitch = -roundToDecimals(fusion.getPitchRadians(),3);
  roll = -roundToDecimals(fusion.getRollRadians(),3);
  //pitch = bfs::rad2deg(pitch);
}