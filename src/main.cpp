#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>
#include <TaskScheduler.h>

#include "motor.h"
#include "Sensors.h"
#include "filtering.h"
#include "map.h"
#include "moving.h"

//**********************************************************************************
// Local Definitions
//**********************************************************************************

// Task period Definitions
// ALL OF THESE VALUES WILL NEED TO BE SET TO SOMETHING USEFUL !!!!!!!!!!!!!!!!!!!!
#define TOF_READ_TASK_PERIOD                5
#define IMU_READ_TASK_PERIOD                13

#define SENSOR_AVERAGE_PERIOD               50
#define LOCATION_TASK_PERIOD                10

#define MOVEMENT_TASK_PERIOD                5
#define SET_MOTOR_TASK_PERIOD               31

#define CHECK_WATCHDOG_TASK_PERIOD          40

// Task execution amount definitions
// -1 means indefinitely
#define TOF_READ_TASK_NUM_EXECUTE          -1
#define IMU_READ_TASK_NUM_EXECUTE          -1

#define SENSOR_AVERAGE_NUM_EXECUTE         -1
#define LOCATION_TASK_NUM_EXECUTE          -1

#define MOVEMENT_TASK_NUM_EXECUTE          -1
#define SET_MOTOR_TASK_NUM_EXECUTE         -1

#define CHECK_WATCHDOG_TASK_NUM_EXECUTE    -1

// Pin deffinitions
#define IO_POWER  49

// Serial deffinitions
#define BAUD_RATE 9600

const byte SX1509_ADDRESS = 0x3F;
SX1509 io; // Create an SX1509 object to be used throughout

//**********************************************************************************
// Task Scheduler and Tasks
//**********************************************************************************

/* The first value is the period, second is how many times it executes
   (-1 means indefinitely), third one is the callback function */

// Tasks for reading sensors 
Task tRead_TOF(TOF_READ_TASK_PERIOD,              TOF_READ_TASK_NUM_EXECUTE,        &TOF_task);
Task tRead_IMU(IMU_READ_TASK_PERIOD,              IMU_READ_TASK_NUM_EXECUTE,        &IMU_task);

Task tSensor_average(SENSOR_AVERAGE_PERIOD,       SENSOR_AVERAGE_NUM_EXECUTE,       &filterTask);
Task tLocate_task(LOCATION_TASK_PERIOD,           LOCATION_TASK_NUM_EXECUTE,        &poseTask);

// Task to set the motor speeds and direction
Task tMove_task(MOVEMENT_TASK_PERIOD,             MOVEMENT_TASK_NUM_EXECUTE,        &movementTask);
Task tSet_motor(SET_MOTOR_TASK_PERIOD,            SET_MOTOR_TASK_NUM_EXECUTE,       &motorTask);

// Tasks to scan for weights and collection upon detection

// Tasks to search for bases and unload weights

// Tasks to check the 'watchdog' timer (These will need to be added in)
//Task tCheck_watchdog(CHECK_WATCHDOG_TASK_PERIOD, CHECK_WATCHDOG_TASK_NUM_EXECUTE, &check_watchdog);

Scheduler taskManager;

void task_init() {
  // This is a class/library function. Initialise the task scheduler
  taskManager.init();

  // Add tasks to the scheduler
  /*Sensors*/
  taskManager.addTask(tRead_TOF);
  taskManager.addTask(tRead_IMU);

  /*Background tasks*/
  taskManager.addTask(tSensor_average);
  taskManager.addTask(tLocate_task);

  /*actuators*/
  taskManager.addTask(tMove_task);
  taskManager.addTask(tSet_motor);

  //enable the tasks
  tRead_TOF.enable();
  tRead_IMU.enable();
  tSensor_average.enable();
  tLocate_task.enable();
  tMove_task.enable();
  tSet_motor.enable();
  //tCheck_watchdog.enable();

  Serial.println("Tasks have been initialised \n");
  delay(1000);
}

void setup() {

  pinMode(IO_POWER, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(IO_POWER, 1);                 //Enable IO power on main CPU board

  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);

  io.begin(SX1509_ADDRESS);

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  task_init();

  map_init();

  initMotors();
  sensorInit(&io);
  filteringInit();
}

void loop() {


  taskManager.execute();    //execute the scheduler
}