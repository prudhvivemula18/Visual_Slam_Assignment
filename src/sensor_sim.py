import numpy as np
import matplotlib.pyplot as plt

def generate_sensor_data(n=100):
    true_distance = 5.0  # meters
    noise_std = 0.3
    readings = true_distance + np.random.normal(0, noise_std, n)
    np.savetxt("output/sensor_readings.txt", readings)
    return readings

if __name__ == "__main__":
    data = generate_sensor_data()
    plt.plot(data)
    plt.title("Simulated Sensor Readings")
    plt.xlabel("Time")
    plt.ylabel("Distance (m)")
    plt.show()
