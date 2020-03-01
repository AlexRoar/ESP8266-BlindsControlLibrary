/*
  Shadow.h - Library for controling shadows through steppers
  Copyright (c) 2006 John Doe.  All right reserved.
*/

#include <ESP8266WiFi.h>
#include <AccelStepper.h>
#include <Arduino.h>
#include <Shadow.h>
#include <cppQueue.h>

Shadow::Shadow(int port0, int port1, int port2, int port3, int steps, int fullSteps, int triggerPort, String id) {
    if (triggerPort != -1)
        pinMode(triggerPort, INPUT);
    this->reversed = 1;
    this->noWork = true;
    this->position = 0;
    this->triggerPort = triggerPort;
    this->singleLoop = 3;
    this->steps = steps;
    this->id = id;
    this->fullSteps = fullSteps;
    this->speed = 350.0;
    this->acceleration = 90.0;
    this->maxSpeed = this->speed;
    this->motor = AccelStepper(steps, port0, port2, port1, port3);
    this->motor.setAcceleration(this->acceleration);
    this->motor.setMaxSpeed(this->maxSpeed);
    this->motor.setSpeed(this->speed);
    this->motor.setCurrentPosition(0);

    path def;
    def.position = 0;
    def.acceleration = this->acceleration;
    def.speed = this->speed;
    def.tillTriger = false;
    this->setTarget(def);
    this->stop();
}

Shadow::Shadow() {}

void Shadow::fullUp() {
    bool trigger = this->triggerPort != -1;
    this->addToQue(0, this->getSpeed(), this->getAcceleration(), trigger);
}

void Shadow::fullDown() {
    this->addToQue(this->fullSteps, this->getSpeed(), this->getAcceleration(), false);
}

int Shadow::getPosition() {
    return this->position;
}

void Shadow::setPosition(int pos) {
    this->position = pos;
}

void Shadow::calibrate(float speed /*= 1*/, float finalSpeed /*= 0.5*/) {
    // this->setSpeed(speed);
    // int currentPos = this->getPosition();

    // if (currentPos != -1)
    // {
    //     this->makeSteps(int(fullSteps * 0.15));
    // }

    // while (!this->isTriggered() && !this->getPosition() == 0)
    // {
    //     this->makeSteps(singleLoop * (-1));
    // }
    // this->setPosition(0);
    // this->makeSteps(steps);

    // this->setSpeed(finalSpeed);
    // int smallerStep = int(this->singleLoop * (-0.5));
    // if (smallerStep == 0)
    //     smallerStep = 1;
    // while (!this->isTriggered())
    // {
    //     this->makeSteps(smallerStep);
    // }
    // this->setPosition(0);
}

void Shadow::setSpeed(float speed) {
    this->speed = speed;
    this->maxSpeed = speed;
    this->motor.setSpeed(speed);
    this->motor.setMaxSpeed(this->maxSpeed);
}

void Shadow::reverse(int reversed) {
    this->reversed = reversed;
}

void Shadow::setTarget(path target) {
    this->currentJob = target;
}

bool Shadow::isTriggered() {
    if (this->triggerPort == -1)
        return false;
    return digitalRead(this->triggerPort) == HIGH;
}

void Shadow::loop() {
    this->motor.run();
    this->retreiveWork();
    this->updatePosition();
    if (!this->noWork) {
        if (this->currentJob.tillTriger && this->isTriggered()) {
            this->currentJob.tillTriger = false;
            this->noWork = true;
            this->motor.stop();
            return;
        }
        if (this->currentJob.tillTriger) {
            this->motor.move(this->singleLoop * this->reversed * this->currentJob.direction);
        }
    }
    if (!this->motor.run()) {
        this->noWork = true;
        this->motor.disableOutputs();
        this->currentState = 2;
    } else {
        this->printDebug();
        if (this->motor.distanceToGo() * this->reversed > 0)
            this->currentState = 1;
        else {
            this->currentState = 0;
        }
    }
}

void Shadow::retreiveWork() {
    if (this->noWork) {
        if (this->queue.isEmpty())
            return;
        else {
            this->noWork = false;
            path task;
            this->queue.pop(&task);
            this->setSpeed(task.speed);
            this->setAcceleration(task.acceleration);
            if (task.tillTriger) {
                this->motor.move(this->singleLoop * this->reversed * task.direction);
            } else {
                this->motor.moveTo(task.position * this->reversed);
            }
            this->motor.enableOutputs();
            this->setTarget(task);
        }
    }
}

void Shadow::printDebug() {
    if (this->debug) {
        path *target = this->getTarget();
        Serial.print("*******\n");
        Serial.print(this->getId() + ": target->");
        Serial.print(target->position);
        Serial.print("; pos->");
        Serial.print(100 - this->getPositionPercent());
        Serial.print(" (");
        Serial.print(this->getPosition());
        Serial.print(")");
        Serial.print("\n");
    }
}

path *Shadow::getTarget() {
    return &this->currentJob;
}

void Shadow::setAcceleration(float acceleration) {
    this->acceleration = acceleration;
    this->motor.setAcceleration(acceleration);
}

float Shadow::getSpeed() {
    return this->speed;
}

void Shadow::addTargetPercent(float percent) {
    int pos = int(percent * float(this->fullSteps) / 100.00);
    this->addTargetPosition(pos);
}

void Shadow::addTargetPosition(int pos) {
    this->addToQue(pos, this->getSpeed(), this->getAcceleration(), false);
    this->noWork = false;
}

void Shadow::setTargetPercent(float percent) {
    int pos = int(percent * float(this->fullSteps) / 100.00);
    this->setTargetPosition(pos);
}

void Shadow::setTargetPosition(int pos) {
    this->stop();
    this->addToQue(pos, this->getSpeed(), this->getAcceleration(), false);
    this->noWork = false;
}

void Shadow::stop() {
    this->clearQueue();
    this->motor.stop();
    this->noWork = true;
}

String Shadow::getId() {
    return this->id;
}

int Shadow::getPositionPercent() {
    if (this->fullSteps == 0)
        return 0;
    float all = float(this->fullSteps);
    return int((float(this->getPosition()) / all) * 100.0);
}

int Shadow::queueLength() {
    return this->queue.getCount();
}

bool Shadow::jobExists() {
    return !this->noWork;
}

void Shadow::clearQueue() {
    this->queue.clean();
    this->queue.flush();
}

int Shadow::getState() {
    return this->currentState;
}

void Shadow::updatePosition() {
    this->setPosition(this->motor.currentPosition() * this->reversed);
}

void Shadow::addToQue(int position, float speed, float acceleration, bool tillTriger) {
    path point;
    point.speed = speed;
    point.position = position;
    point.acceleration = acceleration;
    point.tillTriger = tillTriger;
    if (this->getPosition() < position)
        point.direction = 1;
    else {
        point.direction = -1;
    }
    this->noWork = false;
    this->queue.push(&point);
}

float Shadow::getAcceleration() {
    return this->acceleration;
}

void Shadow::setDebug(bool debug) {
    this->debug = debug;
}

