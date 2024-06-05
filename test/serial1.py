import serial
import time

# Set up serial connection (adjust 'COM3' to the port your Arduino is connected to)
ser = serial.Serial('COM3', 9600, timeout=1)
time.sleep(2)  # Wait for connection to establish

 # Open a text file for writing data
<<<<<<< HEAD:src/serial1.py
with open("20ul_pcr.txt_2", "w") as file:
=======
with open("Data/20uldroplet_108um_20W.txt", "w") as file:
>>>>>>> 7bddc3f13ecbfa1e63979bd4b78386660d39045a:test/serial1.py
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



