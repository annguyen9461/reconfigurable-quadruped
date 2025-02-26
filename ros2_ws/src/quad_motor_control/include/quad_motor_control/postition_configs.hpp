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

#endif // POSITION_CONFIGS_HPP