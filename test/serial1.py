import serial
import time

# Set up serial connection (adjust 'COM3' to the port your Arduino is connected to)
ser = serial.Serial('/dev/tty.usbserial-1120', 9600, timeout=1)
time.sleep(2)  # Wait for connection to establish

 # Open a text file for writing data
with open("Data/Benchtop_experiment_rampingratecontrol_12_11_v2.txt", "w") as file:
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



