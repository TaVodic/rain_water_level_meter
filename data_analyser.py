import serial
import matplotlib.pyplot as plt
import time

# Parameters
com_port = 'COM10'
baud_rate = 115200
expected_lines = 512

try:
    with serial.Serial(com_port, baud_rate, timeout=2) as ser:
        print(f"Opened {com_port} at {baud_rate} baud.")

        time.sleep(2)  # Let the MCU initialize

        # Send just the dot character
        ser.write(b'\n')
        print("Sent '\n'")

        # Read and parse lines
        data_list = []
        while len(data_list) < expected_lines:
            line = ser.readline().decode('utf-8').strip()
            if line.isdigit():
                data_list.append(int(line))
                #print(f"Received: {line}")
            else:
                #print(f"Skipped: {line}")
                #print(len(data_list))
                pass            
                
        addr = data_list[0]
        print("Address: ", addr)     

        while data_list and data_list[-1] == 0:
            data_list.pop()        
            
        data_list = data_list[1:-2]

        distance = [] # initialize empty list

        for x in data_list:
            dist = round(x / 1000000 * 343 / 2 * 100, 1)
            distance.append(dist)
            print(dist)

        # Plot
        plt.figure(figsize=(10, 4))
        plt.plot(distance, marker='o', linestyle='-', markersize=2)
        plt.title("Received Data from COM Port")
        plt.xlabel("Sample Index")
        plt.ylabel("Value [cm]")
        plt.grid(True)
        plt.tight_layout()
        plt.show()

except serial.SerialException as e:
    print(f"Serial error: {e}")
except Exception as ex:
    print(f"Unexpected error: {ex}")
