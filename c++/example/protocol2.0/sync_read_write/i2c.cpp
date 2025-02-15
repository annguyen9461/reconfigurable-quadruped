// https://www.kernel.org/doc/Documentation/i2c/smbus-protocol

#include <linux/i2c-dev.h>  // ioctl(file, I2C_SLAVE, addr);
#include <cstdio>           // C-style printing & string formatting
#include <cstdlib>          // System calls & exit handling
#include <iostream>         // cout, cerr
#include <cstdint>          //ie. uint8_t (Fixed-width integer types)
#include <fcntl.h>          // open device files (file control options)
#include <unistd.h>         // read, write, and close files
#include <sys/ioctl.h>      // low-level I2C commands

int write_register(int file, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    if (write(file, buf, 2) != 2) {
        std::cerr << "Failed to write register 0x" << std::hex << (int)reg << std::endl;
        return -1;
    }
    return 0;
}

int read_register(int file, uint8_t reg) {
    uint8_t data;

    // write the register address to tell the device which register to read from
    if (write(file, &reg, 1) != 1) {
        std::cerr << "Failed to write register address: 0x" << std::hex << (int)reg << std::endl;
        return -1;
    }

    // read from the device, which will return the value stored in that register
    if (read(file, &data, 1) != 1) {
        std::cerr << "Failed to read from register: 0x" << std::hex << (int)reg << std::endl;
        return -1;
    }
    return data;
}

int16_t read_16bit_register(int file, uint8_t reg_low, uint8_t reg_high) {
    int16_t low = read_register(file, reg_low);
    int16_t high = read_register(file, reg_high);
    if (low == -1 || high == -1) return -1;
    return (high << 8) | low; // combine them into a signed 16-bit value
}

int main() {
    int file;
    int adapter_nr = 1; // use /dev/i2c-1
    char filename[20];

    snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
    file = open(filename, O_RDWR);
    if (file < 0) {
        std::cerr << "Failed to open I2C bus\n";
        return 1;
    }

    int addr = 0x6A; // LSM330DHCX I2C address
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        std::cerr << "Failed to set I2C address\n";
        close(file);
        return 1;
    }

    // read WHO_AM_I register (0x0F)
    int who_am_i = read_register(file, 0x0F);
    if (who_am_i >= 0) {
        std::cout << "WHO_AM_I register: 0x" << std::hex << who_am_i << std::endl;
    }

    while (1) {
        // enable gyroscope data (CTRL1_XL = 0x10)
        write_register(file, 0x10, 0x60);
        // enable accelerometer data (CTRL2_G = 0x11)
        write_register(file, 0x11, 0x60);

        sleep(1); // wait for sensor to initialize

        int16_t gyro_x = read_16bit_register(file, 0x22, 0x23);
        int16_t gyro_y = read_16bit_register(file, 0x24, 0x25);
        int16_t gyro_z = read_16bit_register(file, 0x26, 0x27);

        int16_t accel_x = read_16bit_register(file, 0x28, 0x29);
        int16_t accel_y = read_16bit_register(file, 0x2A, 0x2B);
        int16_t accel_z = read_16bit_register(file, 0x2C, 0x2D);

        std::cout << "Gyro (dps) X: " << (int16_t)gyro_x * (250.0 / 32768.0)
            << "  Y: " << (int16_t)gyro_y * (250.0 / 32768.0)
            << "  Z: " << (int16_t)gyro_z * (250.0 / 32768.0) << std::endl;

        std::cout << "Accel (m/s²) X: " << (int16_t)accel_x * (2.0 / 32768.0) * 9.81
            << "  Y: " << (int16_t)accel_y * (2.0 / 32768.0) * 9.81
            << "  Z: " << (int16_t)accel_z * (2.0 / 32768.0) * 9.81 << std::endl;
    }
    
    close(file);
    return 0;
}
