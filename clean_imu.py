import os
import time

file_path = "./data/imu_data.txt"

def clean_imu_file(file_path, max_lines=200, lines_to_remove=150):
    if os.path.exists(file_path):
        with open(file_path, "r") as file:
            lines = file.readlines()

        if len(lines) > max_lines:
            print(f"File has {len(lines)} lines, cleaning up...")
            # Giữ lại phần cuối của file (từ lines_to_remove trở đi)
            remaining_lines = lines[lines_to_remove:]

            with open(file_path, "w") as file:
                file.writelines(remaining_lines)
            
            print(f"Removed {lines_to_remove} lines. File now has {len(remaining_lines)} lines.")
        else:
            print(f"File has {len(lines)} lines. No need to clean.")

    else:
        print(f"File {file_path} not found.")

if __name__ == "__main__":
    while True:
        clean_imu_file(file_path)
        time.sleep(2)  # Chờ 2 giây trước khi kiểm tra lại
