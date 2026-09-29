#include "IMU.h"

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

//2 possible addresses A (0x28) and B (0x29)
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_A, &Wire1);

sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;

static float gyroBias = 0;

static int prevRead = 0;

static volatile bool imuHasData = false;   //FIX: true once a real heading has been read

void calibrate_gyroBias(int num_samples) {
  float sum = 0;
  for (int i = 0; i < num_samples; i++) {
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    sum += (float)angVelocityData.gyro.z;
    delay(5);
  }
  gyroBias = sum/num_samples;
}

void IMU_init() {
  /* Initialise the sensor */
  //FIX: IMUPLUS = gyro + accel fusion only. The default (NDOF) also uses the
  //magnetometer, whose heading jumps near the drive motors / steel in the arena.
  //IMUPLUS heading is relative (starts ~0), which is what the map wants.
  if (!bno.begin(OPERATION_MODE_IMUPLUS))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  Wire1.setClock(400000);   //FIX: Wire1 was left at 100 kHz (only Wire was set to 400 kHz)

  calibrate_gyroBias(10);

  delay(1000);
}

void printIMUEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  switch (event->type) {
  case SENSOR_TYPE_ACCELEROMETER:
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    break;
  
  case SENSOR_TYPE_ORIENTATION:
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
    break;
  
  case SENSOR_TYPE_MAGNETIC_FIELD:
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
    break;

  case SENSOR_TYPE_GYROSCOPE:
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
    break;
  
  case SENSOR_TYPE_ROTATION_VECTOR:
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
    break;

  case SENSOR_TYPE_LINEAR_ACCELERATION:
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    break;

  case SENSOR_TYPE_GRAVITY:
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    break;

  default:
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}

void IMU_task() {
  //FIX: only read what we use. It used to do 5 I2C reads every 5 ms
  //(euler, gyro, linear accel, accel, gravity) = most of the CPU time.
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  imuHasData = true;
}

bool IMU_hasData() {return imuHasData;}

float getHeading() {return (float)orientationData.orientation.x;}


//does not work
float intergrateIMU() {
  int now = micros();
  float dt = float(now - prevRead)/1000000;
  prevRead = now;
  float omega = (float)angVelocityData.gyro.z - gyroBias;
  return omega * dt;
}