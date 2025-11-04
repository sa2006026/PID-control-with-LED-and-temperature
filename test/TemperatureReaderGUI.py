import tkinter as tk
from tkinter import ttk
import serial
import time
import threading

class TemperatureReaderGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Temperature Reader")
        
        # Serial connection
        self.ser = None
        self.connected = False
        self.reading = False
        
        # Create main frame
        self.connection_frame = ttk.LabelFrame(root, text="Connection")
        self.connection_frame.grid(row=0, column=0, padx=2, pady=2, sticky="ew")
        
        # Raw data display
        self.raw_data_frame = ttk.LabelFrame(root, text="Temperature Data")
        self.raw_data_frame.grid(row=1, column=0, padx=2, pady=2, sticky="ew")
        
        # Create a frame for raw data text and scrollbar
        self.raw_data_container = ttk.Frame(self.raw_data_frame)
        self.raw_data_container.grid(row=0, column=0, sticky="nsew")
        
        self.raw_data_text = tk.Text(self.raw_data_container, width=40, height=10)
        self.raw_data_text.grid(row=0, column=0, padx=2, pady=2, sticky="nsew")
        
        # Add scrollbar
        scrollbar = ttk.Scrollbar(self.raw_data_container, command=self.raw_data_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.raw_data_text.config(yscrollcommand=scrollbar.set)
        
        # Connection controls
        self.port_label = ttk.Label(self.connection_frame, text="Port:")
        self.port_label.grid(row=0, column=0, padx=2, pady=2)
        
        self.port_entry = ttk.Entry(self.connection_frame)
        self.port_entry.insert(0, "/dev/cu.usbserial-1120")  # Change this to your port
        self.port_entry.grid(row=0, column=1, padx=2, pady=2)
        
        self.connect_button = ttk.Button(self.connection_frame, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=2, padx=2, pady=2)
        
        # Control buttons
        self.start_button = ttk.Button(self.raw_data_frame, text="On", command=self.start_reading)
        self.start_button.grid(row=1, column=0, padx=2, pady=2)
        
        self.stop_button = ttk.Button(self.raw_data_frame, text="Off", command=self.stop_reading)
        self.stop_button.grid(row=1, column=1, padx=2, pady=2)
        self.stop_button.config(state='disabled')  # Initially disabled
        
        # Status bar
        self.status_label = ttk.Label(root, text="Not Connected", relief=tk.SUNKEN)
        self.status_label.grid(row=2, column=0, columnspan=3, sticky="ew", padx=2, pady=2)

    def toggle_connection(self):
        if not self.connected:
            try:
                port = self.port_entry.get()
                self.ser = serial.Serial(port, 9600, timeout=1)
                time.sleep(2)  # Wait for the connection to establish
                self.connected = True
                self.connect_button.config(text="Disconnect")
                self.status_label.config(text="Connected")
                self.start_button.config(state='normal')  # Enable start button when connected
            except Exception as e:
                self.status_label.config(text=f"Error: {str(e)}")
        else:
            if self.ser:
                self.stop_reading()  # Stop reading if active
                self.ser.close()
            self.connected = False
            self.connect_button.config(text="Connect")
            self.status_label.config(text="Not Connected")
            self.start_button.config(state='disabled')
            self.stop_button.config(state='disabled')

    def start_reading(self):
        if self.connected and not self.reading:
            self.reading = True
            self.raw_data_text.delete(1.0, tk.END)  # Clear previous data
            
            # Start reading temperature in a separate thread
            threading.Thread(target=self.read_temperature, daemon=True).start()
            self.status_label.config(text="Temperature Reading Started")
            self.start_button.config(state='disabled')
            self.stop_button.config(state='normal')

    def stop_reading(self):
        self.reading = False
        self.status_label.config(text="Temperature Reading Stopped")
        self.start_button.config(state='normal')
        self.stop_button.config(state='disabled')

    def read_temperature(self):
        while self.reading and self.connected:
            try:
                # Send command to read temperature
                self.ser.write(b'R')  # Assuming 'R' is the command to read temperature
                self.ser.flush()
                
                if self.ser.in_waiting:
                    data = self.ser.readline().decode().strip()
                    if data:
                        # Update raw data display
                        self.raw_data_text.insert(tk.END, f"{data}\n")
                        self.raw_data_text.see(tk.END)
                
            except Exception as e:
                print(f"Reading error: {str(e)}")
                break
            
            time.sleep(1)  # Adjust the delay as needed

def main():
    root = tk.Tk()
    app = TemperatureReaderGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main() 