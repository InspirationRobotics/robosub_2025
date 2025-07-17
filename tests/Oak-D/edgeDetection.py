import cv2
import numpy as np

# ======= Parameters for tuning =======
path = r"C:\Users\chase\OneDrive\Pictures\Screenshots\Screenshot 2025-07-14 144721.png"
blur_kernel_size = (5, 5)
sobel_ksize = 3               # Sobel kernel size (must be 1,3,5,7)
gradient_thresh = 100         # Threshold on gradient magnitude for edge detection
min_contour_area = 500        # Minimum area of contour to keep
max_contour_area = 1e6        # Maximum area (optional)
# =====================================

# Load image (grayscale)
img = cv2.imread(path)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Blur to reduce noise
blurred = cv2.GaussianBlur(gray, blur_kernel_size, 0)

# Compute Sobel gradients
grad_x = cv2.Sobel(blurred, cv2.CV_64F, 1, 0, ksize=sobel_ksize)
grad_y = cv2.Sobel(blurred, cv2.CV_64F, 0, 1, ksize=sobel_ksize)

# Compute gradient magnitude
grad_mag = cv2.magnitude(grad_x, grad_y)

# Convert to 8-bit image for thresholding
grad_mag_uint8 = cv2.convertScaleAbs(grad_mag)

# Threshold gradient magnitude to get edges
_, edges = cv2.threshold(grad_mag_uint8, gradient_thresh, 255, cv2.THRESH_BINARY)

# Find contours on thresholded edges
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Draw bounding boxes on original image
boxed = img.copy()
for cnt in contours:
    area = cv2.contourArea(cnt)
    if min_contour_area <= area <= max_contour_area:
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(boxed, (x, y), (x + w, y + h), (0, 255, 0), 2)

# Show results
cv2.imshow('Original', img)
cv2.imshow('Gradient Magnitude', grad_mag_uint8)
cv2.imshow('Edges from Sobel', edges)
cv2.imshow('Detected Boxes', boxed)
cv2.waitKey(0)
cv2.destroyAllWindows()
