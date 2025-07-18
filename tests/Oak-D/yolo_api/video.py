import cv2
import os
from .bounding_box import BoundingBox

class Video:    
    """This class handles video file operations, including reading frames and splitting the video into individual frames."""
    def __init__(self, file_path):
        self.file_path = file_path
        self.cap = cv2.VideoCapture(file_path)
        
        # Check if the video file was opened successfully
        if not self.cap.isOpened():
            raise ValueError(f"Could not open video file: {file_path}")
        
        self.frame_rate = self.cap.get(cv2.CAP_PROP_FPS)                #CAP_PROP_FPS to get the frame rate
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT)) # Total number of frames in the video
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))        #CAP_PROP_FRAME_WIDTH to get the width of the video
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))      #CAP_PROP_FRAME_HEIGHT to get the height of the video
        print(f"Loaded video: {file_path}")
        print(f"Frame rate: {self.frame_rate}")
        print(f"Total frames: {self.total_frames}")
        print(f"Resolution: {self.width}x{self.height}")
        
    def split_to_frames(self, output_folder=None, return_frames=True, save_frames=False): 
        """
        Splits video into frames.
        Args:
            output_folder (str): Where to save images. If None, don't save.
            return_frames (bool): If True, returns a list of Frame objects.
            save_frames (bool): If True, saves frames as image files.
        Returns:
            list of Frame objects if return_frames is True; otherwise, None.
        """
        if save_frames and output_folder is not None: # Check if output_folder is provided for saving frames
            if not os.path.exists(output_folder):     # Check if the output folder exists
                os.makedirs(output_folder)            # Create the output folder if it doesn't exist
            frame_num = 0                             # Loop through the frames of the video
            frames = []                               # List to store the frames

        while True:
            ret, frame = self.cap.read()                  # Read a frame from the video
            if not ret:                                   # If the frame is not read successfully, break the loop
                break
            if return_frames:                             # If return_frames is True, create a Frame object
                frames.append(Frame(frame, frame_num))    # Append the frame to the list of frames
            if save_frames and output_folder is not None: # If save_frames is True and output_folder is provided
                frame_path = os.path.join(output_folder, f"frame_{frame_num:04d}.jpg") # Format the frame number with leading zeros
                cv2.imwrite(frame_path, frame)               # Save the frame as an image file
            frame_num += 1                                   # Increment the frame number
        self.cap.release()                                   # Release the video capture object to frees up resources
        print(f"Extracted {frame_num} frames to {output_folder}")
        
        return frames if return_frames else None # Return the list of Frame objects if return_frames is True, otherwise return None

class Frame:
    """This class represents a single frame of video, allowing for image saving and display."""
    def __init__(self, image_data, frame_number):
        self.image_data =image_data
        self.frame_number = frame_number
        self.bounding_boxes = []  # List to store bounding boxes (detections) for this frame

    def save_image(self, output_path):
        """Saves the image data to a file."""
        cv2.imwrite(output_path, self.image_data) # Save the image data to a file

    def show_image(self, window_name="Frame"):
        """Displays the image in a window."""
        cv2.imshow(window_name, self.image_data) # Show the image in a window
        cv2.waitKey(0) # Wait for a key press to close the window
        cv2.destroyAllWindows() 
