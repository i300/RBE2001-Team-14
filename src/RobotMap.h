#include "Utilities.h"

#ifndef ROBOTMAP_H
#define ROBOTMAP_H

#define DEBUG 1

/* GLOBAL */
static const uint8 ROBOT_ID = 21;

/* ROD GRABBER */
static const int8 PIN_SENSOR_POT = A11;

static const int8 PIN_MOTOR_GRABBER = 4;
static const int8 PIN_SERVO_GRABBER = 5;

/* DRIVE TRAIN */
static const int8 PIN_MOTOR_LEFT = 6;
static const int8 PIN_MOTOR_RIGHT = 7;

static const int8 PIN_SENSOR_ALIGNMENT = 22;

/* LINE SENSOR */
static const int8 NUM_SENSORS = 8;
static const int8 NUM_SAMPLES_PER_SENSOR = 4;

static const int8 PIN_SENSOR_LINE1 = A0;
static const int8 PIN_SENSOR_LINE2 = A1;
static const int8 PIN_SENSOR_LINE3 = A2;
static const int8 PIN_SENSOR_LINE4 = A3;
static const int8 PIN_SENSOR_LINE5 = A4;
static const int8 PIN_SENSOR_LINE6 = A5;
static const int8 PIN_SENSOR_LINE7 = A6;
static const int8 PIN_SENSOR_LINE8 = A7;
static const int8 PIN_LINESENSOR_EMITTER = 23;

#endif
