# big diagram
import tkinter as tk
from tkinter import ttk
import serial
import time
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class FiveStageThermocyclingGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Five-Stage Thermocycling Control")
        
        # Serial connection
        self.ser = None
        self.connected = False
        
        # Create main frame
        self.connection_frame = ttk.LabelFrame(root, text="Connection")
        self.connection_frame.grid(row=0, column=0, padx=2, pady=2, sticky="ew")
        
        # Connection controls
        self.port_label = ttk.Label(self.connection_frame, text="Port:")
        self.port_label.grid(row=0, column=0, padx=2, pady=2)
        
        self.port_entry = ttk.Entry(self.connection_frame)
        self.port_entry.insert(0, "/dev/cu.wchusbserial1120")  # Change this to your port
        self.port_entry.grid(row=0, column=1, padx=2, pady=2)
        
        self.connect_button = ttk.Button(self.connection_frame, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=2, padx=2, pady=2)
        
        # Cycle number input
        self.cycle_label = ttk.Label(self.connection_frame, text="Cycle Number:")
        self.cycle_label.grid(row=1, column=0, padx=2, pady=2)
        
        self.cycle_entry = ttk.Entry(self.connection_frame)
        self.cycle_entry.insert(0, "1")  # Default cycle number
        self.cycle_entry.grid(row=1, column=1, padx=2, pady=2)

        # Send Cycle Number button
        self.send_cycle_button = ttk.Button(self.connection_frame, text="Send Cycle Number", command=self.send_cycle_number)
        self.send_cycle_button.grid(row=1, column=2, padx=2, pady=2)

        # Thermocycling control frame
        self.thermo_frame = ttk.LabelFrame(root, text="Photothermal ddPCR system")
        self.thermo_frame.grid(row=1, column=0, padx=2, pady=2, sticky="ew")
        
        # Phase controls
        self.temp_entries = []
        self.time_entries = []
        
        for i in range(5):
            ttk.Label(self.thermo_frame, text=f"Phase {i+1} Temp (°C):").grid(row=i, column=0, padx=2, pady=2)
            temp_entry = ttk.Entry(self.thermo_frame)
            temp_entry.grid(row=i, column=1, padx=2, pady=2)
            self.temp_entries.append(temp_entry)

            ttk.Label(self.thermo_frame, text="Hold Time (s):").grid(row=i, column=2, padx=2, pady=2)
            time_entry = ttk.Entry(self.thermo_frame)
            time_entry.grid(row=i, column=3, padx=2, pady=2)
            self.time_entries.append(time_entry)

        # Send values button
        self.send_button = ttk.Button(self.thermo_frame, text="Send Values", command=self.send_values)
        self.send_button.grid(row=5, column=0, columnspan=4, padx=2, pady=2)

        # Start thermocycling button
        self.start_button = ttk.Button(self.thermo_frame, text="Start", command=self.start_thermocycling)
        self.start_button.grid(row=6, column=0, columnspan=2, padx=2, pady=2)

        # Stop thermocycling button
        self.stop_button = ttk.Button(self.thermo_frame, text="Stop", command=self.stop_thermocycling)
        self.stop_button.grid(row=6, column=2, columnspan=2, padx=2, pady=2)

        # Serial output display
        self.output_frame = ttk.LabelFrame(root, text="Serial Output")
        self.output_frame.grid(row=2, column=0, padx=2, pady=2, sticky="ew")

        # Create a Text widget with a larger font size
        self.output_text = tk.Text(self.output_frame, width=60, height=5, font=("Helvetica", 12))  # Set font size to 12
        self.output_text.grid(row=0, column=0, padx=2, pady=2, sticky="nsew")

        # Add scrollbar for output text
        output_scrollbar = ttk.Scrollbar(self.output_frame, command=self.output_text.yview)
        output_scrollbar.grid(row=0, column=1, sticky="ns")
        self.output_text.config(yscrollcommand=output_scrollbar.set)

        self.export_button = ttk.Button(self.output_frame, text="Export Output", command=self.export_output)
        self.export_button.grid(row=1, column=0, columnspan=2, padx=2, pady=2, sticky="ew")
        # Status bar
        self.status_label = ttk.Label(root, text="Not Connected", relief=tk.SUNKEN)
        self.status_label.grid(row=3, column=0, columnspan=3, sticky="ew", padx=2, pady=2)

        # Live plotting setup
        self.x_data = []
        self.y_data = []
        self.fig, self.ax = plt.subplots(figsize=(3, 2.5))  # Set the figure size (width, height) in inches
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)  # Create a canvas for the plot
        self.canvas.get_tk_widget().place(x=635, y=30, width=1100, height=900)  # Use place to position the widget
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=1000)  # Update every second

    def toggle_connection(self):
        if not self.connected:
            try:
                port = self.port_entry.get()
                self.ser = serial.Serial(port, 9600, timeout=1)
                time.sleep(2)  # Wait for the connection to establish
                self.connected = True
                self.connect_button.config(text="Disconnect")
                self.status_label.config(text="Connected")
            except Exception as e:
                self.status_label.config(text=f"Error: {str(e)}")
        else:
            if self.ser:
                self.ser.close()
            self.connected = False
            self.connect_button.config(text="Connect")
            self.status_label.config(text="Not Connected")

    def send_cycle_number(self):
        if self.connected:
            cycle_number = self.cycle_entry.get().strip()  # Get the cycle number
            command_str = f"C{cycle_number}\n"  # Format the command
            self.ser.write(command_str.encode())  # Send the command to Arduino
            self.ser.flush()  # Ensure the command is sent immediately
            self.status_label.config(text=f"Sent Cycle Number: {cycle_number}")

    def export_output(self):
        from tkinter import filedialog
        import datetime
        
        # Generate default filename with timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        default_filename = f"thermocycling_output_{timestamp}.txt"
        
        # Open file dialog
        filename = filedialog.asksaveasfilename(
            initialfile=default_filename,
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
        )
        
        if filename:
            try:
                # Get all text from the output text widget
                output_text = self.output_text.get("1.0", tk.END)
                
                # Write to file
                with open(filename, 'w') as f:
                    f.write(output_text)
                
                self.status_label.config(text=f"Output saved to {filename}")
            except Exception as e:
                self.status_label.config(text=f"Error saving file: {str(e)}")

    def send_values(self):
        if self.connected:
            # Clear previous output
            self.output_text.delete(1.0, tk.END)  # Clear the text area
            
            commands = []
            for i in range(5):
                temp = self.temp_entries[i].get().strip()  # Strip any whitespace
                hold_time = self.time_entries[i].get().strip()  # Strip any whitespace
                
                # Only add valid entries
                if temp and hold_time:  # Ensure both temp and hold time are provided
                    commands.append(f"{temp},{hold_time}")
                else:
                    commands.append("0,0")  # Default to 0 if not provided

            # Join commands and format the command string
            command_str = "F" + ";".join(commands) + "\n"  # Example command format
            self.ser.write(command_str.encode())  # Send the command to Arduino
            self.ser.flush()  # Ensure the command is sent immediately
            self.status_label.config(text=f"Sent Values: {command_str.strip()}")

    def start_thermocycling(self):
        if self.connected:
            self.ser.write(b"START\n")  # Send START command to Arduino
            self.status_label.config(text="Started Thermocycling")
            # Start reading serial output in a separate thread
            threading.Thread(target=self.read_serial_output, daemon=True).start()  # Start reading serial output

    def stop_thermocycling(self):
        if self.connected:
            self.ser.write(b"STOP\n")  # Send STOP command to Arduino
            self.status_label.config(text="Stopped Thermocycling")

    def read_serial_output(self):
        while self.connected:
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    self.output_text.insert(tk.END, f"{line}\n")
                    self.output_text.see(tk.END)
                    self.root.after(0, self.extract_data, line)  # Use after to call extract_data safely

    def extract_data(self, line):
        try:
            print(f"Raw line: {line}")  # Print the raw line for debugging
            parts = line.split(' ')
            print(f"Split parts: {parts}")  # Print the split parts for debugging
            
            time_part = parts[1]  # Extract time
            temperature_part = parts[3]  # Extract temperature
            
            # Convert to float
            time_value = float(time_part)
            temperature_value = float(temperature_part)

            # Append data for plotting
            self.x_data.append(time_value)
            self.y_data.append(temperature_value)

            # Limit x and y data to the last 100 points
            self.x_data = self.x_data[-100:]
            self.y_data = self.y_data[-100:]

            print(f"Extracted time: {time_value}, temperature: {temperature_value}")  # Print extracted values for debugging

        except (ValueError, IndexError) as e:
            print(f"Error parsing line: {line} - {str(e)}")  # Print error message for debugging

    def moving_average(self, data, window_size):
        if len(data) < window_size:
            return data  # Return original data if not enough points
        return [sum(data[i:i + window_size]) / window_size for i in range(len(data) - window_size + 1)]

    def update_plot(self, frame):
        self.ax.clear()  # Clear the axis

        # Apply moving average to smooth the y_data
        window_size = 5  # You can adjust this value for more or less smoothing
        smoothed_y_data = self.moving_average(self.y_data, window_size)

        # Adjust x_data to match the length of smoothed_y_data
        smoothed_x_data = self.x_data[-len(smoothed_y_data):]  # Get corresponding x values

        # Plot the smoothed data and connect the points with lines
        self.ax.plot(smoothed_x_data, smoothed_y_data, label='Temperature Input', linestyle='-')  # Connect points with lines
        self.ax.set_title('Live Temperature Input', fontsize=5)  # Set title font size
        self.ax.set_xlabel('Time (s)', fontsize=3)  # Set x-label font size
        self.ax.set_ylabel('Temperature (°C)', fontsize=3)  # Set y-label font size
        self.ax.tick_params(axis='both', labelsize=3)

        # Auto-scaling for x-axis
        if smoothed_x_data:
            self.ax.set_xlim(min(smoothed_x_data), max(smoothed_x_data))  # Set x-axis limits based on data

        # Auto-scaling for y-axis
        if smoothed_y_data:
            self.ax.set_ylim(min(smoothed_y_data) - 1, max(smoothed_y_data) + 1)  # Set y-axis limits based on data with some padding

        self.ax.legend(fontsize=3)  # Set legend font size
        self.ax.grid()
        self.canvas.draw()  # Update the canvas

def main():
    root = tk.Tk()
    root.geometry("1180x515")  # Set the window size to 1000 width and 600 height
    app = FiveStageThermocyclingGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()