import cv2
import numpy as np

class CV:
    def __init__(self):
        self.last_result = None

    def run(self, frame):
        output = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2)

        # Hough Circle Transform
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=50,
            param1=100,
            param2=30,
            minRadius=10,
            maxRadius=100
        )

        result = {
            "detected": False,
            "center": None,
            "radius": None,
            "confidence": 0.0
        }

        if circles is not None:
            circles = np.uint16(np.around(circles))
            # Assume the first (strongest) circle is the target
            for i in circles[0, :1]:
                center = (i[0], i[1])
                radius = i[2]

                # Draw the circle and center
                cv2.circle(output, center, radius, (0, 255, 0), 2)
                cv2.circle(output, center, 2, (0, 0, 255), 3)

                # Populate result
                result["detected"] = True
                result["center"] = center
                result["radius"] = radius
                result["confidence"] = 1.0  # Hough doesn’t give direct score, so we default to 1.0 for now
                break

        self.last_result = result
        return result, output


if __name__ == "__main__":
    cv = CV()
    cap = cv2.VideoCapture(1)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        result, img_viz = cv.run(frame)

        if img_viz is not None:
            cv2.imshow("viz", img_viz)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
