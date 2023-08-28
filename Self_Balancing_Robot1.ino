#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

int in_1 = 9;
int in_2 = 6;
int in_3 = 5;
int in_4 = 3;
int ENA = 11;
int ENB = 10;

float traveltime, error, presentTime, previousTime, presentError, previousError;
float distance, kp, ki, kd, pid_p, pid_i, pid_d, dT, dE, PID;
int dKal = 0;          //desired distance from obstacle
int sped, CPID;

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode (in_1, OUTPUT);
  pinMode (in_2, OUTPUT);
  pinMode (in_3, OUTPUT);
  pinMode (in_4, OUTPUT);
  pinMode (ENA, OUTPUT);
  pinMode (ENB, OUTPUT);

  previousTime = millis();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

void loop() {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

  //  Serial.print("AccRoll:");
  //  Serial.print(roll); Serial.print("  gyroRoll:");
  //  Serial.print(gyroXangle); Serial.print(" compRoll:");
  //  Serial.print(compAngleX); Serial.print(" kalRoll:");
  //  Serial.print(kalAngleX);
  //
  //  Serial.print(" Pitch: ");
  //  Serial.print(pitch); Serial.print(" gyroPitch:");
  //  Serial.print(gyroYangle); Serial.print(" compPitch:");
  //  Serial.print(compAngleY);
  //Serial.print(" kalPitch:");
  Serial.print(kalAngleY);
  Serial.print("                  ");

  //#if 0 // Set to 1 to print the temperature
  //  Serial.print("\t");
  //
  //  double temperature = (double)tempRaw / 340.0 + 36.53;
  //  Serial.print(temperature); Serial.print("\t");
  //#endif

  //Serial.print("\r\n");

  errorCalc();
  Pid_p(2);
  presentTime = millis(); //saving the time this line of code is run as "PresentTime" for calculation in Pid_i.
  Pid_i(0.0001);
  Pid_d(4);
  PID = pid_p + pid_i + pid_d; //Total PID value
  PID = abs(PID);   //using absolute modulus on PID to eliminate negatives
  //  Serial.println("  ");
  //Serial.print("PID : ");
  //Serial.print(PID);
  //Serial.print("  ");

  CPID = constrain(PID, 0, 40);
  sped = map(CPID, 0, 40, 242, 255);
  //Serial.print(" CPID: ");
  //Serial.print(CPID);
  //Serial.print("  speed: ");
  Serial.println(sped);

  if ( error > 0.3) {
    reverse();
  }
  if (error < 0.25) {
    forward();
  }
  if (error > 45) {
    stopp();
  }
  if (error < -45) {
    stopp();
  }
//  if ( sped == 0 || sped == 2 || sped == -2){
//    stopp();
//  {
}


























void errorCalc() {
  error = kalAngleX - dKal;  //error is "current distance from obstacle minus desired distance (WanValue)".
  //Serial.print("Error :");
  //Serial.print(error); 
  //Serial.print(" ;   ");
}

void Pid_p(float kp) {
  pid_p = kp * error;
  //  Serial.print("Pid_p : ");
  //  Serial.print(pid_p);
  //  Serial.print(" ; ");
}

void Pid_i(float ki) {
  dT = (presentTime - previousTime) / 1000;  //dT(change in time) is the difference between the current presentTime
  //and the current previousTime whenever this line of code is run.
  //Dividing by 1000 to convert it to microseconds.
  presentTime = previousTime;     //now setting the current presentTime as the new previousTime for future recalculation.
  pid_i += (error * dT * ki);
  //  Serial.print("Pid_i : ");
  //  Serial.print(pid_i);
  //  Serial.print(" ; ");
}

void Pid_d (float kd) {
  presentError = error;                //setting presenterror as the value of the output gotten from errorCalc().
  dE = presentError - previousError;   //dE(change in error value) is the difference between the current error value gotten
  //(presentError) and the current previousError whenever this line of code is run.
  previousError = presentError;        //now setting the current presentError as the new previousError for future recalculation.
  pid_d = kd * (dE / dT);
  //  Serial.print("Pid_d : ");
  //  Serial.println(pid_d);
}

void forward() {
  //ALL WHEELS ClockWise

  digitalWrite(in_1, LOW);
  digitalWrite (in_2, HIGH);
  digitalWrite (in_3, HIGH);
  digitalWrite (in_4, LOW);
  analogWrite (ENA, sped);
  analogWrite (ENB, sped);
}

void reverse() {
  //ALL WHEELS CounterClosckWise
  digitalWrite (in_1, HIGH);
  digitalWrite (in_2, LOW);
  digitalWrite (in_3, LOW);
  digitalWrite (in_4, HIGH);
  analogWrite (ENA, sped);
  analogWrite (ENB, sped);
}

void stopp() {
  //ALL WHEELS Stop
  digitalWrite (in_1, LOW);
  digitalWrite (in_2, LOW);
  digitalWrite (in_3, LOW);
  digitalWrite (in_4, LOW);
  analogWrite (ENA, sped);
  analogWrite (ENB, sped);
}
