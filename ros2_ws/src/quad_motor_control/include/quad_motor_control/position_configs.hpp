#ifndef POSITION_CONFIGS_HPP
#define POSITION_CONFIGS_HPP

#define NUM_MOTORS 12
// Home for walking
int home_tiptoe[NUM_MOTORS + 1] = {0, 
  2745,  // [ID:1]
  2187,  // [ID:2]
  3062,  // [ID:3]
  1343,  // [ID:4]
  1890,  // [ID:5]
  1025,  // [ID:6]
  2752,  // [ID:7]
  2190,  // [ID:8]
  3072,  // [ID:9]
  2429,  // [ID:10]
  1864,  // [ID:11]
  1050   // [ID:12]
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

int aligned_before_rolling[NUM_MOTORS + 1] = {0, 
  2045, 2053, 3049, 2054, 2035, 1014, 2044, 2047, 3071, 3051, 2043, 1056
};

int home_tiptoe_thin[NUM_MOTORS + 1] = {0, 
  2207, 2325, 3053, 1818, 1789, 1020, 2226, 2299, 3070, 2833, 1786, 1049
};

// WALK TO CIR
int walk_to_cir1[NUM_MOTORS + 1] = {0, 
    2045, 1637, 3059, 2052, 2435, 1017, 2045, 1983, 2726, 3051, 2085, 1396
};

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

#endif // POSITION_CONFIGS_HPP