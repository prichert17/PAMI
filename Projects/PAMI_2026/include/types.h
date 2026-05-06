#ifndef TYPES_H
#define TYPES_H

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Enums
enum RobotState { STATE_WAIT, STATE_DELAY, STATE_GAME, STATE_END, STATE_MANUAL, STATE_TEST, STATE_ERROR };
enum TeamColor { COLOR_BLUE, COLOR_YELLOW };

// Structures
struct RobotPose {
    float x;
    float y;
    float theta;
    float voltage;
};

// Objets globaux partagés (déclarés ici, définis dans main.cpp)
extern SemaphoreHandle_t xPoseMutex;
extern RobotState state;
extern TeamColor teamColor;
extern bool lowBattery;
extern bool tofObstacleDetected;

#endif