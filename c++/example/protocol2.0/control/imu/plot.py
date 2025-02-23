import pandas as pd
import plotly.express as px

# Load CSV
df = pd.read_csv("blue point to ground/imu_high_res_data1.csv")

# Plot Gyroscope Data
fig = px.line(df, x='Timestamp', y=['Gyro_X_dps', 'Gyro_Y_dps', 'Gyro_Z_dps'], title='Gyroscope Data Over Time')
fig.show()

# Plot Accelerometer Data
fig = px.line(df, x='Timestamp', y=['Accel_X_mps2', 'Accel_Y_mps2', 'Accel_Z_mps2'], title='Accelerometer Data Over Time')
fig.show()
