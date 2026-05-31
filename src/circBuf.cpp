#include <stdint.h>
#include "stdlib.h"
#include <vector>
#include "circBuf.h"

struct circBuf_t {
    uint16_t size;
    uint16_t windex;
    uint16_t rindex;
    uint16_t* data;
};

uint16_t *bufferInit(uint16_t size, circBuf_t Buffer) {
    Buffer.size = size;
    Buffer.data = new uint16_t[Buffer.size]();
    Buffer.rindex = 0;
    Buffer.windex = 0;
}