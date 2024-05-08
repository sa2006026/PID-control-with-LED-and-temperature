import serial
import time

# Set up serial connection (adjust 'COM3' to the port your Arduino is connected to)
ser = serial.Serial('COM3', 9600, timeout=1)
time.sleep(2)  # Wait for connection to establish

 # Open a text file for writing data
with open("20uldroplet_108um_20W.txt", "w") as file:
    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                print(line)  # Print to console (optional)
                file.write(line + '\n')
                file.flush()  # Ensure data is written to the file
    except KeyboardInterrupt:
        print("Data collection stopped")
        ser.close()  # Close the serial connection when done



