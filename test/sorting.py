def filter_multiples_of_five(filename):
    filtered_data = []
    
    with open(filename, 'r') as file:
        for index, line in enumerate(file, start=1):  # start=1 for 1-based indexing
            # Print line number and content for debugging
            print(f"Reading line {index}: {line.strip()}")
            
            # Keep the line if the row number is a multiple of 5
            if index % 5 == 0:
                filtered_data.append(line.strip())
                print(f"Added line {index} to filtered data")
    
    return filtered_data

# Example usage
filename = './test/data.txt'  # Make sure the path is correct
filtered_values = filter_multiples_of_five(filename)

print("\nFiltered values:")
# Print filtered results
for value in filtered_values:
    print(value)

# Optionally save to new file
with open('filtered_data_3.txt', 'w') as file:
    for value in filtered_values:
        file.write(f"{value}\n")
