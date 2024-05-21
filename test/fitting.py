import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
from sklearn.metrics import r2_score

# Define the function to fit
def linear_func(x, Temp):
    intercept = 26.6  # Fixed intercept value
    return Temp * x / 32 + intercept

# Define the x and y data
x_data = np.array([0.0006416, 0.003823936, 0.009829312, 0.015629376, 0.0221352, 0.027049856, 0.03233664, 0.038072544, 0.0442704, 0.049993472, 0.055536896, 0.061247136, 0.067162688, 0.072988416, 0.078442016, 0.08199648])
y_data = np.array([27.1, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 99])

# Set the initial guess for the parameters
initial_guess = [1.0]  # Replace with your initial guess for Temp

# Perform the curve fit
popt, pcov = curve_fit(linear_func, x_data, y_data, p0=initial_guess)

# Retrieve the fitted parameter
Temp = popt[0]
# Print the fitted parameter and the fixed intercept
print("Temp:", Temp)
print("Intercept:", 26.6)

# Generate x values for the fitted curve
x_fit = np.linspace(min(x_data), max(x_data), 100)

# Calculate the y values for the fitted curve
y_fit = linear_func(x_fit, Temp)

# Plot the original data points
plt.scatter(x_data, y_data, label='Data')

# Plot the fitted curve with adjusted intercept
plt.plot(x_fit, linear_func(x_fit, Temp), 'r-', label='Fitted Curve')

# Calculate R-squared value
y_pred = linear_func(x_data, Temp)
r2 = r2_score(y_data, y_pred)

# Set the x and y axis labels
plt.xlabel('x')
plt.ylabel('Temperature')

# Add a legend
plt.legend()

# Add R-squared value as a text annotation
rsquared_text = f'R-squared: {r2:.4f}'
plt.annotate(rsquared_text, xy=(0.05, 0.7), xycoords='axes fraction')

# Print the linear equation with the fitted parameters
print(f"Linear Equation: y = {Temp:.4f} * x / 32 + 26.6")

# Show the plot
plt.show()