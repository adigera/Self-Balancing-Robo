#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf for theory

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

const int enA = 5;
const int in1 = 7;
const int in2 = 6;
const int in3 = 8;
const int in4 = 9;
const int enB = 10;
int motorSpeedA = 0;
int motorSpeedB = 0;

float limit = 2000;  // limit on PID output

int Kp = 110  ;
int Kd = 2;

float input = 0;        // value output to the PWM (analog out)

float gyroYrateKpre = 0;    //omega Y


char rx_byte = 0;                           //To iunput via Serial Monitor
String rx_str = "";
boolean not_number = false;

/* Inertial Measurement Unit sensor Data */
double accX, accY, accZ;                     //Current/Present acceleration
double gyroX, gyroY, gyroZ;                  ////Current/Present omega
int16_t tempRaw;

float gyroYrateK1;                            //omega after filtering through kalman
float gyroYrateK;                             //

double gyroXangle, gyroYangle; // Angle calculate using the gyro only , unfiltered, found out by integrating omega from sensor
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data


// Params to set custom equilibrium position in first "calibrationTime" seconds after starting the machine
float offsetAngle = 0;
int calibrationTime = 5;    // time in seconds to take average during
long int t = 1000*calibrationTime;
long int timeSum=0 ;
long int timeCount=0;
float angleSum =0;
long int lastTime=0;


void setup() {
  Serial.begin(115200);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);


  Wire.begin();
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

  delay(1000); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
}

void loop() {


        //Serial.println(); Serial.print("\t");
//  if (Serial.available() > 0) {    // is a character available?
//    rx_byte = Serial.read();       // get the character
//
//    if ((rx_byte >= '0') && (rx_byte <= '9')) {
//      rx_str += rx_byte;
//    }
//    else if (rx_byte == '\n') {
//      // end of string
//      if (not_number) {
//        Serial.println("Not a number");
//      }
//      else {
//        // multiply the number by 2
//        Kd = rx_str.toInt() * 0.1;
//
//      }
//      not_number = false;         // reset flag
//      rx_str = "";                // clear the string for reuse
//    }
//    else {
//      // non-number character received
//      not_number = true;    // flag a non-number
//    }
//  } // end: if (Serial.available() > 0)
//







  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

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
  double gyroYrate1 = gyroY / 131.0; // Convert to deg/s

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

  //  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  //  gyroYangle += gyroYrate * dt;
  gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  gyroYrateK1 = kalmanY.getRate();
  gyroYrateK = (kalmanY.getRate() + gyroYrateKpre) / 2;





if(millis()< t)                                                      // averaging angles for t secs for setting an offset
    {
      timeCount++;
      //long int timeDiff = millis()-lastTime;
      angleSum = angleSum + kalAngleY;
      //lastTime = millis();
      //timeSum = timeSum+timeDiff;   
    }
    
    
  
  
  offsetAngle = angleSum/timeCount;
  Serial.print(timeCount);
  Serial.print(" & offset is ");
  Serial.println(offsetAngle);


  

//PID  start
  
  input = Kp * (kalAngleY-offsetAngle) + Kd * (gyroYrateK) ;
  //Serial.println(kalAngleY+2);
  //Serial.print(" : ");
  //Serial.println(gyroYrateK);
  

  if (input > limit)
  {
    input = limit;
  }
  else if (input < -1 * limit)
  {
    input = -1 * limit;
  }
  if (abs(input) < 2 )
  {
    input = 0;
  }



  if (input > 0 ) {
    // Set Motor A backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    // Set Motor B backward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    // Reverse map declining Y-axis readings (for going backward) from 470-0 to 0-255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(1 * input, 0, 2 * limit, 28, 255);
    motorSpeedB = map(1 * input, 0, 2 * limit, 28, 255);
  }
  else if (input < 0) {
    // Set Motor A forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // Set Motor B forward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  
    // Map increasing Y-axis readings (for going forward) from 550-1023 to 0-255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map( -1*input, 0, 2 * limit, 28, 255);
    motorSpeedB = map( -1*input, 0, 2 * limit, 28, 255);
  }


//Serial.println(motorSpeedA);
  
  if (millis() > t) {
  analogWrite(enA, 1.07* motorSpeedA); // Send PWM signal to motor A
  analogWrite(enB, 0.89* motorSpeedB); // Send PWM signal to motor B
  //Serial.println((kalAngleY));Serial.print("\t");
  //Serial.println(); Serial.print("\t");g
  }
  gyroYrateKpre = gyroYrateK;

  delay(5);
 
}
