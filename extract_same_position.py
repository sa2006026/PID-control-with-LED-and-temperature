import cv2
import numpy as np
import matplotlib.pyplot as plt

def select_roi(image):
    # Convert image to RGB for display
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    # Display the image
    plt.imshow(image_rgb)
    plt.title("Select ROI")
    
    # Let the user select a rectangular region
    roi = plt.ginput(2, timeout=0)
    plt.close()
    
    # Convert ROI points to integers
    roi = [(int(x), int(y)) for x, y in roi]
    return roi

def extract_roi(image, roi):
    # Extract coordinates
    (x1, y1), (x2, y2) = roi
    
    # Ensure correct order of coordinates
    x1, x2 = min(x1, x2), max(x1, x2)
    y1, y2 = min(y1, y2), max(y1, y2)
    
    # Extract the ROI
    return image[y1:y2, x1:x2]

def main():
    # Load the two images
    image1 = cv2.imread('image1.jpg')
    image2 = cv2.imread('image2.jpg')
    
    # Check if images are loaded successfully
    if image1 is None or image2 is None:
        print("Error: Unable to load one or both images.")
        return
    
    # Select ROI from the first image
    roi = select_roi(image1)
    
    # Extract ROI from both images
    roi1 = extract_roi(image1, roi)
    roi2 = extract_roi(image2, roi)
    
    # Display results
    plt.figure(figsize=(12, 4))
    
    plt.subplot(131)
    plt.imshow(cv2.cvtColor(roi1, cv2.COLOR_BGR2RGB))
    plt.title("ROI from Image 1")
    
    plt.subplot(132)
    plt.imshow(cv2.cvtColor(roi2, cv2.COLOR_BGR2RGB))
    plt.title("ROI from Image 2")
    
    plt.subplot(133)
    diff = cv2.absdiff(roi1, roi2)
    plt.imshow(cv2.cvtColor(diff, cv2.COLOR_BGR2RGB))
    plt.title("Difference")
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
