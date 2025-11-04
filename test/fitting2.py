import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

# Your data
x = np.array([0.2, 0.4, 0.8, 0.8, 1, 1.2, 1.4, 1.8, 2, 2.2, 2.4])
y = np.array([29, 30.1, 34.2, 40.8, 47.5, 56.2, 66.1, 77.2, 89.9, 100.5, 113.3])

# Perform linear regression
slope, intercept, r_value, p_value, std_err = stats.linregress(x, y)

# Create the line of best fit
line = slope * x + intercept

# New x values to predict
new_x = np.array([2.6, 2.8, 3.0])
# Calculate predicted y values
predicted_y = slope * new_x + intercept

# Find x when y = 500
target_y = 500
target_x = (target_y - intercept) / slope

# Plot the data and the fitted line
plt.figure(figsize=(10, 6))
plt.scatter(x, y, color='blue', label='Original data points')
plt.plot(x, line, color='red', label=f'Fitted line (y = {slope:.2f}x + {intercept:.2f})')
plt.scatter(new_x, predicted_y, color='green', label='Predicted points')
plt.scatter(target_x, target_y, color='orange', label=f'Target point (y=500)')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Linear Regression Fit with Predictions')
plt.legend()
plt.grid(True)

# Print the results
print(f"Slope: {slope:.2f}")
print(f"Intercept: {intercept:.2f}")
print(f"R-squared: {r_value**2:.4f}")
print("\nPredicted y values:")
for x_val, y_val in zip(new_x, predicted_y):
    print(f"x = {x_val:.1f}: y = {y_val:.2f}")

print(f"\nWhen y = 500, x = {target_x:.2f}")

plt.show()
