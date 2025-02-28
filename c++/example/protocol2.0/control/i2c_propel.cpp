#include <linux/i2c-dev.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fstream>
#include <chrono>
#include <iomanip>

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
    if (write(file, &reg, 1) != 1) {
        std::cerr << "Failed to write register address: 0x" << std::hex << (int)reg << std::endl;
        return -1;
    }
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
    return (high << 8) | low;
}

// Function prototypes
void propel_blue() {
    std::cout << ">>> 🚀 Propel Blue Activated!\n";
}

void propel_yellow() {
    std::cout << ">>> 🚀 Propel Yellow Activated!\n";
}

void incre_momentum_for_blue_propel() {
    std::cout << "⚡ Increasing Momentum for Blue Propel...\n";
}

void incre_momentum_for_yellow_propel() {
    std::cout << "⚡ Increasing Momentum for Yellow Propel...\n";
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

    // Enable gyroscope and accelerometer
    write_register(file, 0x10, 0x60); // Accelerometer
    write_register(file, 0x11, 0x60); // Gyroscope
    sleep(1); // Wait for sensor to initialize

    // Bias offsets
    float accel_z_offset = 0.2;
    float gyro_z_offset = -0.37;


    // Open CSV file for writing
    std::ofstream csvFile("imu_data.csv");
    if (!csvFile.is_open()) {
        std::cerr << "Failed to open CSV file\n";
        close(file);
        return 1;
    }

    // Write CSV headers
    csvFile << "Timestamp,Gyro_X_dps,Gyro_Y_dps,Gyro_Z_dps,Accel_X_mps2,Accel_Y_mps2,Accel_Z_mps2\n";





    auto start_time = std::chrono::high_resolution_clock::now();

    // Thresholds based on data analysis
    const float accel_z_threshold = 9.5;    // m/s² for a successful propel
    const float gyro_y_stability = 6.5;     // rad/s threshold for orientation stability
    const int window_size = 100;            // Number of samples to average (for smoothing)
    
    // Variables for data accumulation
    float accumulated_accel_z = 0;
    float accumulated_gyro_y = 0;
    int sample_count = 0;

    // High-resolution data capture loop
    while (true) {
        // Read gyroscope data
        int16_t gyro_x = read_16bit_register(file, 0x22, 0x23);
        int16_t gyro_y = read_16bit_register(file, 0x24, 0x25);
        int16_t gyro_z = read_16bit_register(file, 0x26, 0x27);

        // Read accelerometer data
        int16_t accel_x = read_16bit_register(file, 0x28, 0x29);
        int16_t accel_y = read_16bit_register(file, 0x2A, 0x2B);
        int16_t accel_z = read_16bit_register(file, 0x2C, 0x2D);

        // Apply scale factors
        // float gyro_dps_x = gyro_x * (250.0 / 32768.0);
        float gyro_dps_y = gyro_y * (250.0 / 32768.0);
        // float gyro_dps_z = (gyro_z * (250.0 / 32768.0)) - gyro_z_offset;

        // float accel_mps2_x = accel_x * (2.0 / 32768.0) * 9.81;
        // float accel_mps2_y = accel_y * (2.0 / 32768.0) * 9.81;
        float accel_mps2_z = ((accel_z * (2.0 / 32768.0)) * 9.81) - accel_z_offset;

        // // Timestamp for each reading
        // auto current_time = std::chrono::high_resolution_clock::now();
        // double timestamp = std::chrono::duration<double>(current_time - start_time).count();

        // Print to console
        // std::cout << "Time: " << std::fixed << std::setprecision(6) << timestamp << "s | "
        //           << "Gyro (dps) X: " << gyro_dps_x << " Y: " << gyro_dps_y << " Z: " << gyro_dps_z << " | "
        //           << "Accel (m/s²) X: " << accel_mps2_x << " Y: " << accel_mps2_y << " Z: " << accel_mps2_z << std::endl;

        // std::cout << "Gyro Raw - X: " << gyro_x << " Y: " << gyro_y << " Z: " << gyro_z << " | "
        // << "Accel Raw - X: " << accel_x << " Y: " << accel_y << " Z: " << accel_z << std::endl;

        // Accumulate data for smoothing
        accumulated_accel_z += accel_mps2_z;
        accumulated_gyro_y += gyro_dps_y;
        sample_count++;
        // std::cout << "Sample Count: " << sample_count << std::endl;

        // Check if enough samples have been collected
        if (sample_count >= window_size) {
            float avg_accel_z = accumulated_accel_z / sample_count;
            float avg_gyro_y = accumulated_gyro_y / sample_count;

            // Determine if BLUE or YELLOW is under based on orientation
            bool blue_under = (avg_accel_z > -6.6 && avg_accel_z < 3.9) && (avg_gyro_y > 0);
            bool yellow_under = (avg_accel_z > -4.8 && avg_accel_z < 6.3) && (avg_gyro_y < 0);


            // Check propulsion conditions
            if (avg_accel_z >= accel_z_threshold) {
                std::cout << "Avg Accel Z: " << avg_accel_z << " exceeds threshold " << accel_z_threshold << "\n";
                if (blue_under) {
                    propel_blue();
                } else if (yellow_under) {
                    propel_yellow();
                }
            }
            else {
                std::cout << "Avg Accel Z: " << avg_accel_z << " below threshold. Increasing momentum.\n";
                // If acceleration isn't enough, try increasing momentum
                if (yellow_under) {
                    incre_momentum_for_blue_propel();
                } else if (blue_under) {
                    incre_momentum_for_yellow_propel();
                }
            }
            // Reset accuculators
            // after the window size is met, instead of prematurely which is immediately after the decision
            accumulated_accel_z = 0;
            accumulated_gyro_y = 0;
            sample_count = 0;
        }

        
        // Write to CSV
        csvFile << std::fixed << std::setprecision(6)
                << timestamp << ","
                << gyro_dps_x << "," << gyro_dps_y << "," << gyro_dps_z << ","
                << accel_mps2_x << "," << accel_mps2_y << "," << accel_mps2_z << "\n";

        csvFile.flush(); // Ensure data is written in real-time


        // Adjust sampling rate (lower value for higher frequency)
        usleep(10000); // 10ms delay (~100Hz sampling rate)
    }

    csvFile.close();
    close(file);
    return 0;
}
