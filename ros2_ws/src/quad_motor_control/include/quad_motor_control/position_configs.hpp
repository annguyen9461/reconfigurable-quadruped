#ifndef POSITION_CONFIGS_HPP
#define POSITION_CONFIGS_HPP

#include <iostream>

#define NUM_MOTORS 12
// Function to copy array
void copy_array(int* dest, const int* src) {
    for (int i = 0; i <= NUM_MOTORS; i++) {
        dest[i] = src[i];
    }
}

const double TICKS_PER_DEGREE = 4096.0 / 360.0;  // ≈ 11.37778
const int UP_DOWN_TICKS_TURNING = static_cast<int>(20 * TICKS_PER_DEGREE);
const int CW_CCW_TICKS_TURNING = static_cast<int>(20 * TICKS_PER_DEGREE);

// Home for walking
int home_tiptoe[NUM_MOTORS + 1] = {0, 
  2745, 2228, 3062, 1343, 1890, 1025, 2752, 2190, 3072, 2429, 1864, 1050
};

int home_tiptoe_thin[NUM_MOTORS + 1] = {0, 
  2207, 2325, 3053, 1818, 1789, 1020, 2226, 2299, 3070, 2833, 1786, 1049
};

// Home for rolling
int perfect_cir[NUM_MOTORS + 1] = {0, 
  2040,  // [ID:1]
  1098,  // [ID:2]
  3081,  // [ID:3]
  2054,  // [ID:4]
  2997,  // [ID:5]
  1007,  // [ID:6]
  2041,  // [ID:7]
  2993,  // [ID:8]
  1045,  // [ID:9]
  3054,  // [ID:10]
  1095,  // [ID:11]
  3091   // [ID:12]
};


// WALK TO CIR
int aligned_before_rolling[NUM_MOTORS + 1];
int walk_to_cir1[NUM_MOTORS + 1];

void initialize_relative_configs() {
    // Start with home_tiptoe as the base
    copy_array(aligned_before_rolling, home_tiptoe);
    copy_array(walk_to_cir1, home_tiptoe);

    // Adjust motor positions relative to home_tiptoe
    // Example: Modify joint positions slightly for alignment before rolling
    aligned_before_rolling[1]  -= 700;  // Adjust ID:1
    aligned_before_rolling[2]  -= 130;  // Adjust ID:2
    aligned_before_rolling[3]  -= 13;   // Adjust ID:3
    aligned_before_rolling[4]  += 711;  // Adjust ID:4
    aligned_before_rolling[5]  += 145;  // Adjust ID:5
    aligned_before_rolling[6]  -= 11;   // Adjust ID:6
    aligned_before_rolling[7]  -= 708;  // Adjust ID:7
    aligned_before_rolling[8]  -= 143;  // Adjust ID:8
    aligned_before_rolling[9]  += 1;    // Adjust ID:9
    aligned_before_rolling[10] += 622;  // Adjust ID:10
    aligned_before_rolling[11] += 179;  // Adjust ID:11
    aligned_before_rolling[12] += 6;    // Adjust ID:12

    // Example: Modify joint positions for transitioning to circular motion
    walk_to_cir1[1]  -= 700;
    walk_to_cir1[2]  -= 550;
    walk_to_cir1[3]  -= 3;
    walk_to_cir1[4]  += 710;
    walk_to_cir1[5]  += 545;
    walk_to_cir1[6]  -= 5;
    walk_to_cir1[7]  -= 705;
    walk_to_cir1[8]  -= 207;
    walk_to_cir1[9]  -= 346;
    walk_to_cir1[10] += 621;
    walk_to_cir1[11] += 221;
    walk_to_cir1[12] += 346;

    std::cout << "Relative configurations initialized!\n";
}

// CIR TO WALK
int cir_to_blue3_180[NUM_MOTORS + 1] = {0, 
    2047, 1041, 3089, 2050, 3043, 996, 2086, 2987, 3082, 3054, 1099, 3098
};
int cir_to_both_blues_180[NUM_MOTORS + 1] = {0, 
    2047, 974, 3096, 2049, 3088, 993, 2086, 2979, 3082, 3054, 1101, 1052
};

// RECOVERY LEFT SIDE ON GROUND
int s_shape_30_out[NUM_MOTORS + 1] = {0, 
    2023, 1458, 3088, 2059, 2640, 977, 2081, 2642, 1018, 3085, 1459, 3099
};
int s_shape_full_90_out[NUM_MOTORS + 1] = {0, 
    2023, 2140, 3071, 2091, 1958, 962, 2064, 2051, 1003, 3087, 2045, 3098
};
int blue3_180[NUM_MOTORS + 1] = {0, 
    2020, 2140, 3069, 2140, 1958, 946, 1997, 2051, 3070, 3097, 2043, 3081
};
int cir_to_yellow_up60[NUM_MOTORS + 1] = {0, 
    2045, 1281, 3096, 2045, 2829, 989, 2103, 2987, 3062, 3053, 1100, 1052
};
int cir_to_yellow_up90[NUM_MOTORS + 1] = {0, 
    2046, 1631, 3097, 2042, 2485, 984, 2105, 2990, 3060, 3053, 1104, 1052
};

int leg4_up[NUM_MOTORS + 1] = {0, 
    2040, 1098, 3081, 2054, 2997, 1007, 2041, 2993, 1045, 3054, 1095 - UP_DOWN_TICKS_TURNING, 3091
};

// Leg movement configurations for turning right (Declared but NOT initialized at global scope)
int leg4_up_right[NUM_MOTORS + 1];
int leg4_turn_right[NUM_MOTORS + 1];
int leg4_down_right[NUM_MOTORS + 1];

int leg3_up_right[NUM_MOTORS + 1];
int leg3_turn_right[NUM_MOTORS + 1];
int leg3_down_right[NUM_MOTORS + 1];

int leg2_up_right[NUM_MOTORS + 1];
int leg2_turn_right[NUM_MOTORS + 1];
int leg2_down_right[NUM_MOTORS + 1];

int leg1_up_right[NUM_MOTORS + 1];
int leg1_turn_right[NUM_MOTORS + 1];
int leg1_down_right[NUM_MOTORS + 1];

// Function to initialize turning right configurations
void initialize_turning_configs_right() {
    // --- Leg 4 Movements ---
    copy_array(leg4_up_right, home_tiptoe);
    leg4_up_right[11] -= UP_DOWN_TICKS_TURNING;  // Up (ID:11)

    copy_array(leg4_turn_right, leg4_up_right);
    leg4_turn_right[10] += CW_CCW_TICKS_TURNING; // Turning Right CW (ID:10)

    copy_array(leg4_down_right, leg4_turn_right);
    leg4_down_right[11] += UP_DOWN_TICKS_TURNING; // Down (ID:11)

    // --- Leg 3 Movements ---
    copy_array(leg3_up_right, leg4_down_right);
    leg3_up_right[8] += UP_DOWN_TICKS_TURNING; // Up (ID:8)

    copy_array(leg3_turn_right, leg3_up_right);
    leg3_turn_right[7] += CW_CCW_TICKS_TURNING; // Turning Right CW (ID:7)

    copy_array(leg3_down_right, leg3_turn_right);
    leg3_down_right[8] -= UP_DOWN_TICKS_TURNING; // Down (ID:8)

    // --- Leg 2 Movements ---
    copy_array(leg2_up_right, leg3_down_right);
    leg2_up_right[5] -= UP_DOWN_TICKS_TURNING; // Up (ID:5)

    copy_array(leg2_turn_right, leg2_up_right);
    leg2_turn_right[4] += CW_CCW_TICKS_TURNING; // Turning Right CW (ID:4)

    copy_array(leg2_down_right, leg2_turn_right);
    leg2_down_right[5] += UP_DOWN_TICKS_TURNING; // Down (ID:5)

    // --- Leg 1 Movements ---
    copy_array(leg1_up_right, leg2_down_right);
    leg1_up_right[2] += UP_DOWN_TICKS_TURNING; // Up (ID:2)

    copy_array(leg1_turn_right, leg1_up_right);
    leg1_turn_right[1] += CW_CCW_TICKS_TURNING; // Turning Right CW (ID:1)

    copy_array(leg1_down_right, leg1_turn_right);
    leg1_down_right[2] -= UP_DOWN_TICKS_TURNING; // Down (ID:2)

    std::cout << "Turning right configurations initialized!\n";
}


#endif // POSITION_CONFIGS_HPP