#include "map.h"
#include "locate.h"

static uint8_t map[Y_MAX][X_MAX];

void update_map(pos_t* pos, int8_t def) {
    map[pos->y][pos->x + X_ZERO] = def;
}

int16_t getPointDef(pos_t* pos) {
    return map[pos->y][pos->x + X_ZERO];
}