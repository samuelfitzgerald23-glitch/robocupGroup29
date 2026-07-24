#ifndef MOVING_H
#define MOVING_H

#include "map.h"

enum moveingState {
    IDLE,
    TURNING,
    MOVEING,
    DONE
};

class PID_t {
    private:
        float kp, ki, kd;
        float integral = 0.0;
        float prevError = 0.0;
        float intMax;

    public:
        float tolerance;

        PID_t(float kp_, float ki_, float kd_, float intMax_, float tolerance_)
            : kp(kp_), ki(ki_), kd(kd_), intMax(intMax_), tolerance(tolerance_) {}

        void reset() {integral = 0.0; prevError = 0.0;}

        float update(float error, float dt) {
            integral += error * dt;
            integral = constrain(integral, 0, intMax);

            float de = (dt > 0.0f) ? (error - prevError) : 0.0f;

            return constrain(kp * error + ki * integral + kd * de, 350, 450);
        }
};

void findTarget();

float findAngle(pos_t* myPose);


bool rotatePID(pos_vector_t* myPose, float dt, PID_t rotate);
bool translatePID(pos_t* currentXY, float dt, PID_t translate);

void movementTask();

#endif //MOVING_H