#include <iostream>
#include <cstring>  // Needed for strerror()
// #include <linux/i2c-dev.h>

#include "i2c/i2c.c"

#define I2C_DEVICE "/dev/i2c-1"
#define ISM330DLC_ADDR 0x6A
#define WHO_AM_I_REG 0x0F

int main() {
    int bus = i2c_open(I2C_DEVICE);

    /* Open i2c bus /dev/i2c-0 */
    if (bus == -1) {
        /* Error process */
        std::cerr << "Failed to open I2C bus" << std::endl;
        return -1;
    }
    return 0;
}
