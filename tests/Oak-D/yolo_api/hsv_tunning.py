import cv2
import numpy as np
import tkinter as tk

video_path = "C:/Users/HOME/Documents/GitHub/CV_data/poles_test_1.mp4" # Path to your video file
# --- HSV Parameter Names, Defaults, and Ranges ---
hsv_params = [
    ("LH1", 0, 0, 179),   ("LS1", 100, 0, 255), ("LV1", 100, 0, 255),
    ("UH1", 10, 0, 179),  ("US1", 255, 0, 255), ("UV1", 255, 0, 255),
    ("LH2", 170, 0, 179), ("LS2", 100, 0, 255), ("LV2", 100, 0, 255),
    ("UH2", 179, 0, 179), ("US2", 255, 0, 255), ("UV2", 255, 0, 255),
]

def process_frame(frame, hsv_values):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower1 = np.array(hsv_values[0:3])
    upper1 = np.array(hsv_values[3:6])
    lower2 = np.array(hsv_values[6:9])
    upper2 = np.array(hsv_values[9:12])
    mask1 = cv2.inRange(hsv, lower1, upper1)
    mask2 = cv2.inRange(hsv, lower2, upper2)
    mask = cv2.bitwise_or(mask1, mask2)
    result = cv2.bitwise_and(frame, frame, mask=mask)
    return result, mask

# --- Tkinter GUI for HSV values ---
class HSVCalibApp:
    def __init__(self, root, params):
        self.root = root
        self.vars = {}
        self.entries = {}
        for i, (name, default, minval, maxval) in enumerate(params):
            tk.Label(root, text=f"{name}:").grid(row=i, column=0, sticky='w')
            var = tk.IntVar(value=default)
            slider = tk.Scale(root, from_=minval, to=maxval, orient=tk.HORIZONTAL, variable=var, command=lambda val, n=name: self.update_entry(n, val))
            slider.grid(row=i, column=1, sticky='ew')
            entry = tk.Entry(root, width=5)
            entry.insert(0, str(default))
            entry.grid(row=i, column=2)
            entry.bind("<Return>", lambda e, n=name: self.entry_update_slider(n))
            entry.bind("<FocusOut>", lambda e, n=name: self.entry_update_slider(n))
            self.vars[name] = var
            self.entries[name] = entry
            # callback for direct entry
        tk.Button(root, text="Apply", command=self.apply).grid(row=len(params), column=0, columnspan=3)
        self.updated = False
        root.grid_columnconfigure(1, weight=1)

    def get(self, name):
        return self.vars[name].get()

    def get_all(self):
        return [self.vars[name].get() for name, _, _, _ in hsv_params]

    def update_entry(self, name, val):
        # Update entry when slider moves
        entry = self.entries[name]
        if entry.get() != str(val):
            entry.delete(0, tk.END)
            entry.insert(0, str(val))
        self.updated = True

    def entry_update_slider(self, name):
        try:
            val = int(self.entries[name].get())
        except:
            val = self.vars[name].get()
        minval = hsv_params[[n for n,_,_,_ in hsv_params].index(name)][2]
        maxval = hsv_params[[n for n,_,_,_ in hsv_params].index(name)][3]
        val = max(minval, min(val, maxval))
        self.vars[name].set(val)
        self.updated = True

    def apply(self):
        self.updated = True

root = tk.Tk()
root.title("HSV Calibration GUI")
app = HSVCalibApp(root, hsv_params)

# --- OpenCV preview window ---
cv2.namedWindow("Result", cv2.WINDOW_NORMAL)
cap = cv2.VideoCapture(video_path)
paused = False
frame_idx = 0

ret, frame = cap.read()
if not ret:
    print("Video could not be read or is empty.")
    exit(1)
current_frame = frame.copy()
frame_idx = 1  # Start at first frame

while True:
    root.update()
    hsv_values = app.get_all()
    result, mask = process_frame(current_frame, hsv_values)
    cv2.imshow("Original", current_frame)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", result)

    key = cv2.waitKey(30) & 0xFF

    if key == 27:  # ESC
        break

    elif key == ord(' '):  # Pause/play toggle
        paused = not paused

    elif key == ord('d'):  # Step forward one frame
        paused = True
        ret, frame = cap.read()
        if ret:
            current_frame = frame.copy()
            frame_idx += 1
        else:
            print("End of video.")

    elif key == ord('a'):  # Step backward one frame
        paused = True
        seek_frame = max(0, frame_idx - 2)
        cap.set(cv2.CAP_PROP_POS_FRAMES, seek_frame)
        ret, frame = cap.read()
        if ret:
            current_frame = frame.copy()
            frame_idx = seek_frame + 1

    elif not paused:
        # In play mode, auto-advance every cycle
        ret, frame = cap.read()
        if ret:
            current_frame = frame.copy()
            frame_idx += 1
        else:
            print("End of video.")
            break

cap.release()
cv2.destroyAllWindows()
root.destroy()

