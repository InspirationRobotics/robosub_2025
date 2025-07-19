import cv2
import numpy as np

path = r"C:\Users\chase\OneDrive\Pictures\Screenshots\Screenshot 2025-07-14 215709.png" # replace with your image path
img = cv2.imread(path)
if img is None:
    raise FileNotFoundError(f"Image not found: {path}")

cv2.imshow("Original", img)

# Step 1: Convert to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imshow("Grayscale", gray)

# Step 2: Histogram Equalization to improve contrast
equalized = cv2.equalizeHist(gray)
cv2.imshow("Histogram Equalized", equalized)

# Step 3.5: Apply Gaussian Blur to smooth the image
blurred = cv2.GaussianBlur(equalized, (5, 5), 0)
cv2.imshow("Blurred", blurred)

# Step 3: Adaptive Thresholding
adaptive_thresh = cv2.adaptiveThreshold(blurred, 255,
                                        cv2.ADAPTIVE_THRESH_MEAN_C,
                                        cv2.THRESH_BINARY_INV,
                                        15, 7)
cv2.imshow("Adaptive Threshold", adaptive_thresh)


# Step 4: Canny edge detection on blurred image
edges = cv2.Canny(adaptive_thresh, 75, 200)
cv2.imshow("Canny Edges", edges)

# Step 5: Probabilistic Hough Line detection
lines = cv2.HoughLinesP(edges,
                        rho=1,
                        theta=np.pi/180,
                        threshold=50,
                        minLineLength=10,
                        maxLineGap=10)

line_img = img.copy()

# Filter lines by verticality (angle near 90 degrees)
if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0:
            angle = 90
        else:
            angle = abs(np.degrees(np.arctan2(dy, dx)))
        # Keep lines close to vertical (e.g., between 80 and 100 degrees)
        if 80 <= angle <= 100:
            cv2.line(line_img, (x1, y1), (x2, y2), (0, 255, 0), 3)

cv2.imshow("Detected Vertical Lines", line_img)

cv2.waitKey(0)
cv2.destroyAllWindows()
