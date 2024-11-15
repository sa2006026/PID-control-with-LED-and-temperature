import cv2
import numpy as np

def calculate_psnr(original, processed):
    mse = np.mean((original.astype(float) - processed.astype(float)) ** 2)
    if mse == 0:
        return float('inf')
    max_pixel = 255.0
    psnr = 20 * np.log10(max_pixel / np.sqrt(mse))
    return psnr

# Load the noisy image
image = cv2.imread('test/droplet.png', cv2.IMREAD_GRAYSCALE)

# Save the original grayscale image
cv2.imwrite('test/droplet_grayscale.png', image)

# Calculate PSNR of the original image (should be infinity or a very large number)
original_psnr = calculate_psnr(image, image)
print(f"PSNR of original image: {original_psnr:.2f} dB")

# Function to apply Gaussian filter and calculate PSNR
def apply_gaussian_and_calculate_psnr(image, kernel_size, sigma):
    smoothed_image = cv2.GaussianBlur(image, (kernel_size, kernel_size), sigma)
    psnr_value = calculate_psnr(image, smoothed_image)
    return smoothed_image, psnr_value

# Apply Gaussian filter with different kernel sizes
kernel_sizes = [3, 5, 7]  # Changed from [1, 2, 3] to [3, 5, 7]
sigma = 0  # Let OpenCV calculate sigma based on kernel size

results = []
for size in kernel_sizes:
    smoothed, psnr_value = apply_gaussian_and_calculate_psnr(image, size, sigma)
    results.append((size, smoothed, psnr_value))

# Display and save results
print(f"Original image saved as 'test/droplet_grayscale.png'")
for size, smoothed, psnr_value in results:
    print(f"Kernel size: {size}x{size}, PSNR: {psnr_value:.2f} dB")
    cv2.imwrite(f'test/smoothed_droplet_k{size}.png', smoothed)

# Display comparison
comparison = np.hstack([image] + [result[1] for result in results])
cv2.imshow('Original vs Smoothed (Increasing Kernel Size)', comparison)
cv2.waitKey(0)
cv2.destroyAllWindows()
