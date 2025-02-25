import pandas as pd
import plotly.graph_objs as go
import os

# Paths
csv_folder = os.path.join(os.getcwd(), "blue_towards_ground")
output_folder = os.path.join(os.getcwd(), "plots_html")

# Create output folder if it doesn't exist
os.makedirs(output_folder, exist_ok=True)

# List CSV files
csv_files = [f for f in os.listdir(csv_folder) if f.endswith('.csv')]

# Process each CSV file
for csv_file in csv_files:
    csv_path = os.path.join(csv_folder, csv_file)
    df = pd.read_csv(csv_path)

    # Downsample for Performance (optional)
    downsample_rate = 10
    df = df.iloc[::downsample_rate, :].reset_index(drop=True)

    # Create Plotly Figure
    fig = go.Figure()

    # Add Gyro Data Traces
    fig.add_trace(go.Scatter(x=df['Timestamp'], y=df['Gyro_X_dps'], mode='lines', name='Gyro_X_dps'))
    fig.add_trace(go.Scatter(x=df['Timestamp'], y=df['Gyro_Y_dps'], mode='lines', name='Gyro_Y_dps'))
    fig.add_trace(go.Scatter(x=df['Timestamp'], y=df['Gyro_Z_dps'], mode='lines', name='Gyro_Z_dps'))

    # Add Accel Data Traces
    fig.add_trace(go.Scatter(x=df['Timestamp'], y=df['Accel_X_mps2'], mode='lines', name='Accel_X_mps2'))
    fig.add_trace(go.Scatter(x=df['Timestamp'], y=df['Accel_Y_mps2'], mode='lines', name='Accel_Y_mps2'))
    fig.add_trace(go.Scatter(x=df['Timestamp'], y=df['Accel_Z_mps2'], mode='lines', name='Accel_Z_mps2'))

    # Customize Layout
    fig.update_layout(
        title=f"IMU Data Plot for {csv_file}",
        xaxis_title="Time (seconds)",
        yaxis_title="Sensor Values",
        template='plotly_white',
        legend=dict(orientation='h', y=-0.2)
    )

    # Save Plot to HTML
    output_filename = os.path.splitext(csv_file)[0] + ".html"
    html_path = os.path.join(output_folder, output_filename)
    fig.write_html(html_path, include_plotlyjs='cdn')

    print(f"✅ Plot saved: {html_path}")

print("\n🎉 All plots generated in 'plots_html' folder.")
