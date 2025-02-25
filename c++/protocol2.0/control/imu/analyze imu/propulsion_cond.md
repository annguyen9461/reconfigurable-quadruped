# Propulsion Conditions for Blue and Yellow Legs

This document outlines the specific thresholds and conditions under which the **Blue** and **Yellow** legs should propel or increase momentum.

## Propulsion Decision Table

| **Leg Color** | **Condition**                       | **Acceleration Threshold (m/s²)** | **Orientation Threshold (degrees)** | **Action**                          |
|---------------|-------------------------------------|------------------------------------|-------------------------------------|-------------------------------------|
| **Blue**      | Blue leg under, sufficient accel.   | > 9.5                              | ±30                                 | Propel                             |
| **Blue**      | Blue leg under, insufficient accel. | ≤ 9.5                              | ±30                                 | Increase momentum for Blue         |
| **Blue**      | Blue leg not under                  | -                                  | -                                   | Do not Propel                      |
| **Yellow**    | Yellow leg under, sufficient accel. | > 9.7                              | ±35                                 | Propel                             |
| **Yellow**    | Yellow leg under, insufficient accel.| ≤ 9.7                             | ±35                                 | Increase momentum for Yellow       |
| **Yellow**    | Yellow leg not under                | -                                  | -                                   | Do not Propel                      |

## Explanation of Thresholds

- **Acceleration Threshold:**  
  - Blue requires at least **9.5 m/s²** to successfully propel.  
  - Yellow requires at least **9.7 m/s²** to successfully propel.

- **Orientation Threshold:**  
  - Blue can propel when its orientation is within ±30°.  
  - Yellow requires a ±35° orientation range for stability.

- **Increase Momentum:**  
  When thresholds are not met, the system should trigger either:
  - `incre_momentum_for_blue_propel()`
  - `incre_momentum_for_yellow_propel()`

These functions help build the necessary momentum before the next propel attempt.

---

## Usage

1. **Determine which leg is under.**  
2. **Compare IMU data against thresholds.**  
3. **Call the appropriate function based on the table.**

