#include <Wire.h>
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};

void kalman_1d(float *KalmanState, float *KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  // Predict
  *KalmanState = *KalmanState + 0.004 * KalmanInput;
  *KalmanUncertainty = *KalmanUncertainty + 0.004 * 0.004 * 4 * 4;

  // Update
  float KalmanGain = *KalmanUncertainty / (*KalmanUncertainty + 3 * 3);
  *KalmanState = *KalmanState + KalmanGain * (KalmanMeasurement - *KalmanState);
  *KalmanUncertainty = (1 - KalmanGain) * (*KalmanUncertainty);
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.read();
  Wire.read();
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  // Convert gyroscope readings to rates (degrees per second)
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  // Swap AccX and AccY to change axes
  AccX = (float)AccYLSB / 4096 - 0.3;
  AccY = (float)AccXLSB / 4096 - 0.1;
  AccZ = (float)AccZLSB / 4096 + 0.1;

  // Calculate Roll and Pitch angles using accelerometer data
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * (180 / 3.14159);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * (180 / 3.14159);
}

void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000); // Set the I2C clock speed
  Wire.begin();
  delay(250);
  
  // Wake up the MPU6050
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B); 
  Wire.write(0x00); // Set wake up
  Wire.endTransmission();
  
  // Calibration loop for gyroscope (2000 readings)
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
  LoopTimer = micros();
}

void loop() {
  gyro_signals();

  // Remove the calibration offset from gyroscope readings
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
  
  // Kalman filter for Roll and Pitch
  kalman_1d(&KalmanAngleRoll, &KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  kalman_1d(&KalmanAnglePitch, &KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);

  // Print the Roll and Pitch angles
  Serial.print("Roll Angle [°] ");
  Serial.print(KalmanAngleRoll);
  Serial.print(" Pitch Angle [°] ");
  Serial.println(KalmanAnglePitch);
  
  // Control the loop speed to make it faster and more responsive
  while (micros() - LoopTimer < 1000); // Shorter delay (1ms)
  LoopTimer = micros();
}
