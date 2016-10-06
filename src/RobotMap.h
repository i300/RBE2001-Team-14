#include "Utilities.h"

#ifndef ROBOTMAP_H
#define ROBOTMAP_H

#define DEBUG 1

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

static const int8 PIN_SENSOR_LINE1 = A8;
static const int8 PIN_SENSOR_LINE2 = A9;
static const int8 PIN_SENSOR_LINE3 = A10;
static const int8 PIN_SENSOR_LINE4 = A11;
static const int8 PIN_SENSOR_LINE5 = A12;
static const int8 PIN_SENSOR_LINE6 = A13;
static const int8 PIN_SENSOR_LINE7 = A14;
static const int8 PIN_SENSOR_LINE8 = A15;
static const int8 PIN_LINESENSOR_EMITTER = 23;

#endif
