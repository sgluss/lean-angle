/* MPU9250 Basic Example Code

   mods to get working out of the box (sjr)
  1. choose AD0 below instead of AD1
  2. edit out LCD code
  3. activate magnetometer calibration
  4. add delays for prints
  5. shorten prints

  Corrected wrong hand of magnetometer data in quaternion_update call.

  by: Kris Winer
  date: April 1, 2014
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.
  Modified by Brent Wilkins July 19, 2016

  Demonstrate basic MPU-9250 functionality including parameterizing the register
  addresses, initializing the sensor, getting properly scaled accelerometer,
  gyroscope, and magnetometer data out. Added display functions to allow display
  to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
  Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
  and the Teensy 3.1.

  SDA and SCL should have external pull-up resistors (to 3.3V).
  10k resistors are on the EMSENSR-9250 breakout board.

  Hardware setup:
  MPU9250 Breakout --------- Arduino
  VDD ---------------------- 3.3V
  VDDI --------------------- 3.3V
  SDA ----------------------- A4
  SCL ----------------------- A5
  GND ---------------------- GND
*/

#include "quaternionFilters.h"
#include "MPU9250.h"

#include <stlport.h>
#include <Eigen30.h>
#include <EigenAVR.h>
#include <Eigen/Dense>
#include <EEPROM.h>

using namespace Eigen;

#define AHRS true         // Set to false for basic data read
#define SerialDebug false  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 4;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

int BUTTON_PIN = 7;
int LEFT_LED_PIN = 10;
int CENTER_LED_PIN = 9;
int RIGHT_LED_PIN = 8;

int PITCH_OUTPUT_PIN = 5;
int ROLL_OUTPUT_PIN = 6;
int LAT_G_OUTPUT_PIN = 3;
int LONG_G_OUTPUT_PIN = 11;

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

enum Mode {
  UNCALIBRATED,
  RUNNING,
  CAL_INIT,
  CAL_GRAV_REF_READY,
  CAL_LEANING
};

char* modeNames[] = {"UNCALIBRATED", "RUNNING", "CAL_INIT", "CAL_GRAV_REF_READY", "CAL_LEANING"};

Mode mode = UNCALIBRATED;

void setup()
{
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LEFT_LED_PIN, OUTPUT);
  pinMode(CENTER_LED_PIN, OUTPUT);
  pinMode(RIGHT_LED_PIN, OUTPUT);

  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(115200);

  while (!Serial) {};

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0], 1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1], 1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2], 1); Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3], 1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4], 1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5], 1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);

    if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    //myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    Serial.println("AK8963 mag biases (mG)");
    Serial.println(myIMU.magBias[0]);
    Serial.println(myIMU.magBias[1]);
    Serial.println(myIMU.magBias[2]);

    Serial.println("AK8963 mag scale (mG)");
    Serial.println(myIMU.magScale[0]);
    Serial.println(myIMU.magScale[1]);
    Serial.println(myIMU.magScale[2]);
    //    delay(2000); // Add delay to see results before serial spew of data

    if (SerialDebug)
    {
      Serial.println("Magnetometer:");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }
  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }

  bool got_quat = get_rotation_quaternion();
  if (got_quat) {
    Serial.println("Initiated quaternion, in mode RUNNING");
    mode = RUNNING;
  } else {
    Serial.println("No rotation quaternion, in mode UNCALIBRATED");
    mode = UNCALIBRATED;
  }
}

int oldButtonState = 0;
int buttonState = 0;
int measureCount = 0;

Vector3d grav;
Vector3d down;
unsigned long buttonPress = 0;
unsigned long buttonRelease = 0;
bool doButtonAction = false;
void calibration()
{
  buttonState = digitalRead(BUTTON_PIN);

  if (buttonState == 1 && oldButtonState == 0) {
    Serial.println("Button pressed");
    buttonPress = millis();
  }
  if (buttonState == 0 && oldButtonState == 1) {
    Serial.println("Button released");
    doButtonAction = true;
    buttonRelease = millis();
  }
  oldButtonState = buttonState;

  if (doButtonAction) {
    if (buttonRelease - buttonPress < 3000) {
      Serial.print("Mode switch from "); Serial.print(modeNames[mode]); Serial.print(" to CAL_INIT\n");
      mode = CAL_INIT;
  
      buttonPress = millis();
      buttonRelease = buttonPress;
      doButtonAction = false;
      grav = Vector3d(0.0, 0.0, 0.0);
      return;
    } else {
      reset_eeprom();
      Serial.print("Mode switch from "); Serial.print(modeNames[mode]); Serial.print(" to UNCALIBRATED\n");
      mode = UNCALIBRATED;
    }
  }
  doButtonAction = false;
  
  switch (mode)
  {
    case UNCALIBRATED: uncalibrated();
      break;
    case RUNNING: running();
      break;
    case CAL_INIT: calInit();
      break;
    case CAL_GRAV_REF_READY: calGravRefReady();
      break;
    case CAL_LEANING: calLeaning();
      break;
  }
}

Matrix3f rotationMatrix;
// http://renderdan.blogspot.com/2006/05/rotation-matrix-from-axis-vectors.html
void setRotationMatrix(Vector3d& right, Vector3d& down, Vector3d& forward) {
  // tait-bryan
  // x+ - forward
  // z+ - down
  // y - side
  // https://stackoverflow.com/questions/18558910/direction-vector-to-rotation-matrix

  rotationMatrix << forward[0], right[0], down[0],
                 forward[1], right[1], down[1],
                 forward[2], right[2], down[2];

  rotationMatrix.col(0).normalize();
  rotationMatrix.col(1).normalize();
  rotationMatrix.col(2).normalize();

  Serial.print("Generated rotation matrix! Determinant: "); Serial.print(rotationMatrix.determinant()); Serial.print("\n");
}

double rotation_q1 = 0.0;
double rotation_q2 = 0.0;
double rotation_q3 = 0.0;
double rotation_q4 = 0.0;
Quaterniond rotationQ = Quaterniond(1.0, 0.0, 0.0, 0.0);
void setRotationQuaternionFromRotationMatrix() {
  Matrix3f mat = rotationMatrix;

  rotation_q1 = sqrt(1.0 + mat(0, 0) + mat(1, 1) + mat(2, 2)) / 2.0;
  double w4 = (4.0 * rotation_q1);
  rotation_q2 = (mat(2, 1) - mat(1, 2)) / w4 ;
  rotation_q3 = (mat(0, 2) - mat(2, 0)) / w4 ;
  rotation_q4 = (mat(1, 0) - mat(0, 1)) / w4 ;
  rotationQ = Quaterniond(rotation_q1, rotation_q2, rotation_q3, rotation_q4);
}

double downMagnitude;
unsigned long time = 0;
void calInit() {
  digitalWrite(LEFT_LED_PIN, LOW);
  digitalWrite(CENTER_LED_PIN, LOW);
  digitalWrite(RIGHT_LED_PIN, LOW);
  digitalWrite(LEFT_LED_PIN, HIGH);

  if (millis() - buttonPress < 2000) {
    return;
  }

  if (measureCount < 1000) {
    grav[0] += myIMU.ax / 1000.0;
    grav[1] += myIMU.ay / 1000.0;
    grav[2] += myIMU.az / 1000.0;
    measureCount += 1;
  } else {
    measureCount = 0;
    down[0] = grav[0];
    down[1] = grav[1];
    down[2] = grav[2];
    downMagnitude = down.norm();

    Serial.print("Gravity magnitude: "); Serial.print(downMagnitude, 4); Serial.print("\n");
    Serial.print("Gravity vector: "); Serial.print(down[0], 6); Serial.print("x, ");
    Serial.print(down[1], 6); Serial.print("y, ");
    Serial.print(down[2], 6); Serial.print("z\n");

    Serial.print("Mode switch from "); Serial.print(modeNames[mode]); Serial.print(" to "); Serial.print(modeNames[CAL_GRAV_REF_READY]); Serial.print("\n");
    mode = CAL_GRAV_REF_READY;
  }
}

// Gravity corrected values
int leanSamples = 0;
void calGravRefReady() {
  digitalWrite(LEFT_LED_PIN, LOW);
  digitalWrite(CENTER_LED_PIN, LOW);
  digitalWrite(RIGHT_LED_PIN, LOW);
  digitalWrite(CENTER_LED_PIN, HIGH);

  Serial.print("Mode switch from "); Serial.print(modeNames[mode]); Serial.print(" to "); Serial.print(modeNames[CAL_LEANING]); Serial.print("\n");
  mode = CAL_LEANING;
  leanSamples = 0;
}


bool wasAdequatelyLeaned = false;
double angleToDown = 0.0;
double gravMagnitude = 0.0;
double LEAN_THRESHOLD = 15 / RAD_TO_DEG; // only read values when grav vector deviates by more than this
double LEAN_SAMPLES_COUNT = 1000.0;
Vector3d sumLeanedGravity = Vector3d(0.0, 0.0, 0.0);
Vector3d right;
Vector3d forward;
void calLeaning() {
  grav[0] = myIMU.ax;
  grav[1] = myIMU.ay;
  grav[2] = myIMU.az;

  gravMagnitude = grav.norm();

  angleToDown = acos(down.dot(grav) / (downMagnitude * gravMagnitude));

  if (angleToDown > LEAN_THRESHOLD) {
    sumLeanedGravity[0] += grav[0] / LEAN_SAMPLES_COUNT;
    sumLeanedGravity[1] += grav[1] / LEAN_SAMPLES_COUNT;
    sumLeanedGravity[2] += grav[2] / LEAN_SAMPLES_COUNT;
    leanSamples += 1;
    if (wasAdequatelyLeaned == false) {
      digitalWrite(RIGHT_LED_PIN, HIGH);
    }
    wasAdequatelyLeaned = true;
  }
  else {
    if (wasAdequatelyLeaned == true) {
      digitalWrite(RIGHT_LED_PIN, LOW);
    }
    wasAdequatelyLeaned = false;
  }
  if (leanSamples >= LEAN_SAMPLES_COUNT) {
    forward = Vector3d(down.cross(sumLeanedGravity));
    right = Vector3d(down.cross(forward));

    setRotationMatrix(right, down, forward);
    setRotationQuaternionFromRotationMatrix();
    store_rotation_quaternion();
    mode = RUNNING;
  }
}

double projectionMagnitude = 0.0;
double angleToProjection = 0.0;

int lastUpdate = millis();
int now;
void uncalibrated() {
  now = millis();
  if (now - lastUpdate < 50) {
    return;
  }
  lastUpdate = now;

  blink_async(LEFT_LED_PIN, 750, 0.5);
  blink_async(CENTER_LED_PIN, 750, 0.5);
  blink_async(RIGHT_LED_PIN, 750, 0.5);
}

void running() {
  now = millis();
  if (now - lastUpdate < 50) {
    return;
  }
  lastUpdate = now;

  update_analog_outputs();

  if (myIMU.roll > -90.0 && myIMU.roll < 90.0) {
    blink_async(LEFT_LED_PIN, 500, (-myIMU.roll) / 45.0);
    blink_async(RIGHT_LED_PIN, 500, myIMU.roll / 45.0);
    blink_async(CENTER_LED_PIN, 500, (45.0 - abs(myIMU.roll)) / 45.0);

  } else {
    blink_async(LEFT_LED_PIN, 250, 0.5);
    blink_async(CENTER_LED_PIN, 250, 0.5);
    blink_async(RIGHT_LED_PIN, 250, 0.5);
  }
}

float rollOutput;
float pitchOutput;
int lastAnalogOutUpdate = 0;
void update_analog_outputs() {
  now = millis();
  if (now - lastAnalogOutUpdate < 10) {
    return;
  }
  lastAnalogOutUpdate = now;

  rollOutput = (255.0 * (myIMU.roll + 90.0)) / 180.0;
  pitchOutput = (255.0 * (myIMU.pitch + 90.0)) / 180.0;

  rollOutput = rollOutput < 0.0 ? 0.0 : rollOutput;
  rollOutput = rollOutput > 255.0 ? 255.0 : rollOutput;
  pitchOutput = pitchOutput < 0.0 ? 0.0 : pitchOutput;
  pitchOutput = pitchOutput > 255.0 ? 255.0 : pitchOutput;
  
  analogWrite(ROLL_OUTPUT_PIN, rollOutput);
  analogWrite(PITCH_OUTPUT_PIN, pitchOutput);
}

// interval: ms
// dutyCycle: 0.0 - 1.0 (1.0 is full on, 0.0 is full off)
void blink_async(int ledPin, int interval, float dutyCycle) {
  dutyCycle = dutyCycle > 1.0 ? 1.0 : dutyCycle;
  dutyCycle = dutyCycle < 0.0 ? 0.0 : dutyCycle;

  time = millis();
  if ((time % interval) > (interval * dutyCycle))
  {
    digitalWrite(ledPin, LOW);
  } else {
    digitalWrite(ledPin, HIGH);
  }
}

void store_rotation_quaternion() {
  int addr = 0;
  EEPROM.put(addr, 0xD34DC0D3);
  addr += sizeof(long);
  EEPROM.put(addr, rotationQ);
  addr += sizeof(Quaterniond);
  Serial.println("Quaternion stored!");
}

bool get_rotation_quaternion() {
  int addr = 0;
  unsigned long flag = 0;
  EEPROM.get(addr, flag);
  Serial.print("Getting rotation quaternion, got flag value: ");Serial.print(flag);Serial.println();
  if (flag != 0xD34DC0D3) {
    return false;
  }
  
  addr += sizeof(long);
  EEPROM.get(addr, rotationQ);
  return true;
}

// reset flag
void reset_eeprom() {
  int addr = 0;
  Serial.println("Clearing EEPROM...");
  EEPROM.put(addr, 0x00000000);
}

double q1;
double q2;
double q3;
double q4;
Quaterniond ahrsQ;
Vector3d euler = Vector3d(0, 0, 0);
double resultQ1;
double resultQ2;
double resultQ3;
double resultQ4;
void runPositionUpdate() {
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s

  // fix handedness of magnetometer system, so it actually works (change sign of mz)

  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, -myIMU.mz, myIMU.deltat);

  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
    {
      if (SerialDebug)
      {
        Serial.println();
        // Print acceleration values in milligs!
        Serial.print("Xa "); Serial.print(1000 * myIMU.ax);
        Serial.print(" mg ");
        Serial.print("Ya "); Serial.print(1000 * myIMU.ay);
        Serial.print(" mg ");
        Serial.print("Za "); Serial.print(1000 * myIMU.az);
        Serial.println(" mg ");

        // Print gyro values in degree/sec
        Serial.print("Xg "); Serial.print(myIMU.gx);
        Serial.print(" d/s ");
        Serial.print("Yg "); Serial.print(myIMU.gy);
        Serial.print(" d/s ");
        Serial.print("Zg "); Serial.print(myIMU.gz);
        Serial.println(" d/s ");

        // Print mag values in degree/sec
        Serial.print("Xm "); Serial.print(myIMU.mx);
        Serial.print(" mG ");
        Serial.print("Ym "); Serial.print(myIMU.my);
        Serial.print(" mG ");
        Serial.print("Zm "); Serial.print(myIMU.mz);
        Serial.println(" mG");

        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
        //       Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
        //       Serial.println(" degrees C");
        //delay(2000);
      }
      myIMU.count = millis();
      digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  else
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;

    // update LCD once per half-second independent of read rate
    if (myIMU.delt_t > 500)
    {
      if (SerialDebug)
      {
        Serial.print("ax = ");  Serial.print((int)1000 * myIMU.ax);
        Serial.print(" ay = "); Serial.print((int)1000 * myIMU.ay);
        Serial.print(" az = "); Serial.print((int)1000 * myIMU.az);
        Serial.println(" mg");

        Serial.print("gx = ");  Serial.print(myIMU.gx, 2);
        Serial.print(" gy = "); Serial.print(myIMU.gy, 2);
        Serial.print(" gz = "); Serial.print(myIMU.gz, 2);
        Serial.println(" deg/s");

        Serial.print("mx = ");  Serial.print((int)myIMU.mx);
        Serial.print(" my = "); Serial.print((int)myIMU.my);
        Serial.print(" mz = "); Serial.print((int)myIMU.mz);
        Serial.println(" mG");

        Serial.print("q0 = ");  Serial.print(*getQ());
        Serial.print(" qx = "); Serial.print(*(getQ() + 1));
        Serial.print(" qy = "); Serial.print(*(getQ() + 2));
        Serial.print(" qz = "); Serial.println(*(getQ() + 3));
      }

      /*
        https://robotics.stackexchange.com/questions/15264/how-to-rotate-a-rotation-quaternion-in-the-body-frame-to-a-rotation-quaternion-i
        here, multiply qs (starting orientation) by qr (rotation) to get qs1 (corrected orientation)
        Create direction vector from: https://stackoverflow.com/questions/18558910/direction-vector-to-rotation-matrix
      */
      ahrsQ = Quaterniond(*getQ(), *(getQ() + 1), *(getQ() + 2), *(getQ() + 3));
      ahrsQ *= rotationQ;

      // Define output variables from updated quaternion---these are Tait-Bryan
      // angles, commonly used in aircraft orientation. In this coordinate system,
      // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
      // x-axis and Earth magnetic North (or true North if corrected for local
      // declination, looking down on the sensor positive yaw is counterclockwise.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the
      // Earth is positive, up toward the sky is negative. Roll is angle between
      // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
      // arise from the definition of the homogeneous rotation matrix constructed
      // from quaternions. Tait-Bryan angles as well as Euler angles are
      // non-commutative; that is, the get the correct orientation the rotations
      // must be applied in the correct order which for this configuration is yaw,
      // pitch, and then roll.
      // For more see
      // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
      // which has additional links.

      // adapted to use eigen quaternion
      myIMU.yaw   = atan2(2.0f * (ahrsQ.x() * ahrsQ.y() + ahrsQ.w()
                                  * ahrsQ.z()), ahrsQ.w() * ahrsQ.w() + ahrsQ.x()
                          * ahrsQ.x() - ahrsQ.y() * ahrsQ.y() - ahrsQ.z()
                          * ahrsQ.z());
      myIMU.pitch = -asin(2.0f * (ahrsQ.x() * ahrsQ.z() - ahrsQ.w()
                                  * ahrsQ.y()));
      myIMU.roll  = atan2(2.0f * (ahrsQ.w() * ahrsQ.x() + ahrsQ.y()
                                  * ahrsQ.z()), ahrsQ.w() * ahrsQ.w() - ahrsQ.x()
                          * ahrsQ.x() - ahrsQ.y() * ahrsQ.y() + ahrsQ.z()
                          * ahrsQ.z());

      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;

      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      //      myIMU.yaw  -= 8.5;
      myIMU.roll *= RAD_TO_DEG;

      //if(SerialDebug)
      {
        Serial.print("YPR: ");
        Serial.print(myIMU.yaw, 1);
        Serial.print(", ");
        Serial.print(myIMU.pitch, 1);
        Serial.print(", ");
        Serial.println(myIMU.roll, 1);

        Serial.print("rate = ");
        Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
        Serial.println(" Hz");
      }
      // With these settings the filter is updating at a ~145 Hz rate using the
      // Madgwick scheme and >200 Hz using the Mahony scheme even though the
      // display refreshes at only 2 Hz. The filter update rate is determined
      // mostly by the mathematical steps in the respective algorithms, the
      // processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
      // an ODR of 10 Hz for the magnetometer produce the above rates, maximum
      // magnetometer ODR of 100 Hz produces filter update rates of 36 - 145 and
      // ~38 Hz for the Madgwick and Mahony schemes, respectively. This is
      // presumably because the magnetometer read takes longer than the gyro or
      // accelerometer reads. This filter update rate should be fast enough to
      // maintain accurate platform orientation for stabilization control of a
      // fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
      // produced by the on-board Digital Motion Processor of Invensense's MPU6050
      // 6 DoF and MPU9150 9DoF sensors. The 3.3 V 8 MHz Pro Mini is doing pretty
      // well!

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)
  } // if (AHRS)
}

unsigned long lastLoop = 0;
void loop()
{
  runPositionUpdate();
  calibration();
}
