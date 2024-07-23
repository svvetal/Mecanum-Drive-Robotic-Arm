import smbus2
import time
import math

# MPU6050 registers and addresses
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_ZOUT_H = 0x47

# Gyroscope sensitivity
GYRO_SENSITIVITY = 131.0  # degrees per second per LSB

# Initialize I2C bus
bus = smbus2.SMBus(1)

# Initialize MPU6050
bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

# Initial time
prev_time = time.time()

# Initial z-axis angle
prev_angle_z = 0.0

# Total angle moved in z-axis
total_angle_z = 0.0

# Main loop
try:
    while True:
        # Read gyro data for z-axis
        gyro_z_raw = bus.read_byte_data(MPU6050_ADDR, GYRO_ZOUT_H)
        if gyro_z_raw > 127:
            gyro_z_raw -= 256  # Convert to signed value if necessary
        gyro_z = gyro_z_raw / GYRO_SENSITIVITY  # Convert to degrees per second

        # Current time
        curr_time = time.time()
        dt = curr_time - prev_time

        # Calculate angle moved in z-axis
        angle_z = gyro_z * dt  # Change in angle in this time step

        # Check if there's a significant change in the z-axis angle
        if abs(angle_z) > 0.01:  # Adjust this threshold as needed
            # Update total angle moved in z-axis
            total_angle_z += angle_z

            # Print angle moved in z-axis
            print("Angle moved in z-axis: {:.2f} degrees".format(total_angle_z))

        print(total_angle_z)

        # Update previous time for next iteration
        prev_time = curr_time

        # Add a delay to control the sampling rate
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Exiting program...")
