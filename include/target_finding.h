#ifndef TARGET_FINDING_H
#define TARGET_FINDING_H

#include "smart_servo.h"
#include "map.h"
#include "Arduino.h"

#define KERNEL_SIZE 3
#define TARGET_GRID_SIZE 20

#define MAX_WEIGHTS_ONBOARD 1

enum goal_t {
    NO_ACTION,
    BASE_ACTION,
    WEIGHT_ACTION
};

pos_t find_target(pos_t myPos, pos_t prev_pos, goal_t& target_type);

#endif //TARGET_FINDING_H