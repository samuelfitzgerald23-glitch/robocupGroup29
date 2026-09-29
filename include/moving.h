#ifndef MOVING_H
#define MOVING_H

#include "map.h"

#define MICRO_TO_SEC 1000000

enum moveingState {
    IDLE,
    TURNING,
    MOVEING,
    DONE
};

//FIX: result of one translatePID step
enum translateResult {
    DRIVING,
    ARRIVED,
    OVERSHOT,       //went past the target - turn back towards it
    NEED_TURN       //heading drifted too far - stop and turn again
};

class PID_t {
    private:
        float kp, ki, kd;
        float integral = 0.0;
        float prevError = 0.0;
        float intMax;
        float constraint;

    public:

        float tolerance;

        PID_t(float kp_, float ki_, float kd_, float intMax_, float constraint_, float tolerance_)
            : kp(kp_), ki(ki_), kd(kd_), intMax(intMax_), constraint(constraint_), tolerance(tolerance_) {}

        void reset() {integral = 0.0; prevError = 0.0;}

        float update(float error, float dt) {
            integral += error * dt;
            integral = constrain(integral, -intMax, intMax);

            float de = (dt > 0.0f) ? (error - prevError)/dt : 0.0f;
            prevError = error;

            //speed is clamped when writing to the motors
            return constrain(kp * error + ki * integral + kd * de, -constraint, constraint);
        }
};

void findTarget();

float findAngle(pos_t* myPose);


bool rotatePID(pos_vector_t* myPose, float dt);
translateResult translatePID(pos_vector_t* myPose, float dt);

void movementTask(float dt);

bool getPathStatus();

bool getPathBlockedStatus();

void setPath(const pos_t* path, int count);

//stop the motors and reset the movement state machine
void stopMovement();

#endif //MOVING_H
