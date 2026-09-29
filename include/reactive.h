#pragma once
//----------------------------------------------------------------------------
// Reactive (no-pathfinding) controller.
//   1 = use reactive bang-bang control   (State.cpp hands over to reactiveTask)
//   0 = original pathfinding state machine
// To disable: comment out / set to 0. Nothing else needs to change.
//----------------------------------------------------------------------------
#define USE_REACTIVE_CONTROL 1

//Which side of the arena the robot starts on (only matters for USE_REACTIVE_CONTROL)
//1 = starts on the LEFT  -> heads out to the right
//0 = starts on the RIGHT -> heads out to the left
#define START_ON_LEFT 1

void reactiveTask();
