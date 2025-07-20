import cv2
import numpy as np

path = r"C:\Users\chase\OneDrive\Pictures\Screenshots\Screenshot 2025-07-14 144721.png"
img = cv2.imread(path)
height, width = img.shape[:2]

# Crop the image
hc = 150
wc = 50
img = img[250:height - hc, wc:width - wc]

cv2.imshow('raw', img)

lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
l, a, b = cv2.split(lab)

# Threshold on L channel to detect dark regions (white = dark)
_, dark_mask = cv2.threshold(l, 70, 255, cv2.THRESH_BINARY_INV)

cv2.imshow('darkmask', dark_mask)

# Max continuous white pixel run per column
max_run_per_column = np.zeros(dark_mask.shape[1], dtype=int)

for col in range(dark_mask.shape[1]):
    column_data = dark_mask[:, col]
    max_run = run = 0
    for val in column_data:
        if val == 255:
            run += 1
            max_run = max(max_run, run)
        else:
            run = 0
    max_run_per_column[col] = max_run

# Optionally filter by max run length
min_run_length = 20
columns_to_keep = max_run_per_column >= min_run_length

# Draw vertical lines on kept columns
marked_img = img.copy()
line_thickness = 4
line_color = (0, 255, 0)

for col_idx, keep in enumerate(columns_to_keep):
    if keep:
        cv2.line(marked_img, (col_idx, 0), (col_idx, marked_img.shape[0] - 1), line_color, thickness=line_thickness)

# Show results
cv2.imshow("Marked Columns (By Max Run)", marked_img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Optional: Print max run info
# print(max_run_per_column)
