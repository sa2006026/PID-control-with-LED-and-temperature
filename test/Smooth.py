import cv2
import numpy as np

# Load the image
image = cv2.imread('test/droplet.png')

# Convert to grayscale if it's a color image
if len(image.shape) == 3:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
else:
    gray = image

# Apply Gaussian blur
blurred = cv2.GaussianBlur(gray, (3, 3), 0)

# Perform unsharp masking
sharpened = cv2.addWeighted(gray, 1.5, blurred, -0.5, 0)

# Enhance contrast
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
enhanced = clahe.apply(sharpened)

# Save or display the result
cv2.imwrite('test/enhanced_droplet.png', enhanced)

# Display original and enhanced images side by side
comparison = np.hstack((gray, enhanced))
cv2.imshow('Original vs Enhanced', comparison)
cv2.waitKey(0)
cv2.destroyAllWindows()