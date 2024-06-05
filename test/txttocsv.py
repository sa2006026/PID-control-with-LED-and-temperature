import re
import csv

# File names
input_filename = 'data(PT100+thermocouple)2.txt'
output_filename = 'temperature_data.csv'

# Regular expression to find temperature and time
pattern = r"Temperature: ([\d.]+) Time: (\d+)"

# Read the input file
with open(input_filename, 'r') as file:
    input_text = file.read()

# Find all matches
matches = re.findall(pattern, input_text)

# Write to CSV
with open(output_filename, 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Temperature', 'Time'])  # Header
    for match in matches:
        writer.writerow(match)

print(f"Data has been written to {output_filename}")
