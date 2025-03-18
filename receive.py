import time
import random

def main():
    file_path = "./data/imu_data.txt"  # File to store IMU data

    print("Starting IMU data simulation...")

    try:
        with open(file_path, "a") as file:  # Open file in append mode
            while True:
                # Simulate data in format "s:4:<random_number>:e"
                random_value = random.randint(0, 100)  # Generate random number
                data = f"s:4:{random_value}:e"
                
                print(f"Generated data: {data}")
                file.write(data + "\n")  # Write to file
                file.flush()  # Ensure data is written immediately

                time.sleep(1)  # Simulate data arriving every second

    except KeyboardInterrupt:
        print("\nStopping IMU data simulation.")

if __name__ == "__main__":
    main()
