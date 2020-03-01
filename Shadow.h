/*
  Shadow.h - Library for controling shadows through steppers
  Copyright (c) 2006 John Doe.  All right reserved.
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <AccelStepper.h>
#include <cppQueue.h>

#ifndef Shadow_h
#define Shadow_h

#define IMPLEMENTATION LIFO

typedef struct {
    int position;
    float speed;
    float acceleration;
    bool tillTriger;
    int direction;
} path;

class Shadow {
public:
    Shadow(int port0, int port1, int port2, int port3, int steps, int fullSteps, int triggerPort = -1, String id = "");

    Shadow();

    void fullUp();

    void fullDown();

    int getPosition();

    void setPosition(int pos);

    void calibrate(float speed = 1, float finalSpeed = 0.5);

    void setSpeed(float);

    void reverse(int);

    void setTarget(path);

    bool isTriggered();

    void loop();

    void addToQue(int position, float speed, float acceleration, bool tillTriger = false);

    float getSpeed();

    void setTargetPercent(float percent);

    void setTargetPosition(int pos);

    void addTargetPercent(float percent);

    void addTargetPosition(int pos);

    void stop();

    String getId();

    int getPositionPercent();

    int queueLength();

    bool jobExists();

    void clearQueue();

    int getState();

    void printDebug();

    void setAcceleration(float acceleration);

    void updatePosition();

    float getAcceleration();

    void setDebug(bool debug);

private:
    bool debug = true;
    float acceleration;
    float maxSpeed;
    int currentState = 2;

    void retreiveWork();

    path *getTarget();

    Queue queue = Queue(sizeof(path), 10, IMPLEMENTATION);
    path currentJob;
    bool noWork = true;
    int port0;
    int port1;
    int port2;
    int port3;
    int steps;
    int fullSteps;
    float speed;
    int position = 0;
    int triggerPort;
    int reversed = 1;
    int singleLoop = 5;
    String id = "";
    AccelStepper motor;;
};

#endif