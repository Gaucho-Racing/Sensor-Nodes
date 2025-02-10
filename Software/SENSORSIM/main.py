import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


fs = 200;
duration = 10;
t = np.linspace(0, duration, fs*duration)

acc_x = 10 * np.sin(2 * np.pi * 0.3 * t) + np.random.normal(0, 0.05, len(t))
acc_y = 10 * np.sin(2 * np.pi * 0.3 * t + np.pi / 2) + np.random.normal(0, 0.05, len(t))
acc_z = 9.81 + np.random.normal(0, 0.1, len(t))  # Including gravity

# Generate gyroscope data (deg/s)
gyro_x = 10 * np.sin(2 * np.pi * 0.2 * t) + np.random.normal(0, 0.5, len(t))
gyro_y = 5 * np.sin(2 * np.pi * 0.3 * t) + np.random.normal(0, 0.5, len(t))
gyro_z = 2 * np.sin(2 * np.pi * 0.1 * t) + np.random.normal(0, 0.2, len(t))

imu_data = pd.DataFrame({
    "time": t,
    "acc_x": acc_x, "acc_y": acc_y, "acc_z": acc_z,
    "gyro_x": gyro_x, "gyro_y": gyro_y, "gyro_z": gyro_z,
})

roll = np.arctan2(acc_y, np.sqrt(acc_x**2 + acc_z**2)) * (180 / np.pi)
pitch = np.arctan2(-acc_x, np.sqrt(acc_y**2 + acc_z**2)) * (180 / np.pi)
yaw = np.arctan2(acc_z, np.sqrt(acc_x**2 + acc_y**2)) * (180 / np.pi)


plt.figure(figsize=(10, 5))
plt.plot(t, pitch, label='yaw')
plt.plot(t, acc_x, label='x')
plt.plot(t, acc_y, label='y')
plt.legend()
plt.title("Simulated Accelerometer Data")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (m/s^2)")
plt.show()
