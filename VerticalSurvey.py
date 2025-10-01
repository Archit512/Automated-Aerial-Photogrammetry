# Import all the required libraries.
import airsim # type: ignore
import time
import os
import numpy as np # type: ignore
import math
from datetime import datetime

class VerticalSurvey:
    """
    A class to handle a multi-drone vertical photogrammetry survey in AirSim.

    This class provides a structured way to fly a lawnmower pattern and
    capture top-down images for photogrammetry.
    """

    def __init__(self, vehicle_names, flight_heights, survey_area, image_spacing, speed, camera_pitch_angle, survey_type="VerticalSurvey"):
        """
        Initializes the survey object with required parameters.

        Args:
            vehicle_names (list): A list of vehicle names (e.g., ["Drone6", "Drone7"]).
            flight_heights (list): A list of flight heights (negative Z values) for each drone.
            survey_area (dict): A dictionary defining the survey boundaries.
            image_spacing (float): The distance in meters between each image capture point.
            speed (float): The speed of the drones in m/s.
            camera_pitch_angle (float): The camera pitch angle in degrees (should be close to -90 for vertical shots).
            survey_type (str): Type of survey for directory naming (default: "VerticalSurvey").
        """
        self.client = airsim.MultirotorClient()
        self.vehicle_names = vehicle_names
        self.flight_heights = flight_heights
        self.survey_area = survey_area
        self.image_spacing = image_spacing
        self.speed = speed
        self.camera_pitch_angle = camera_pitch_angle
        
        # Create unique directory name using current date and time
        current_time = datetime.now()
        if os.path.sep in survey_type:
            # survey_type contains a path, use it directly
            self.survey_directory = survey_type
        else:
            # survey_type is just a name, add timestamp
            self.survey_directory = current_time.strftime(f"{survey_type}_%Y%m%d_%H%M%S")
        
        # Create the directory if it doesn't exist
        if not os.path.exists(self.survey_directory):
            os.makedirs(self.survey_directory)
            print(f"Created survey directory: {self.survey_directory}")

    def connect_and_arm(self):
        """Connects to AirSim and arms all drones."""
        # Connect to virtual environment on airsim, and enable the multirotor drones.
        print("Connecting to AirSim...")
        self.client.confirmConnection()
        for drone in self.vehicle_names:
            self.client.enableApiControl(True, drone)
            print(f"Enabling API control for {drone}...")
        
        # Arm the drones to respond to flight commands.
        print("Arming all drones...")
        for drone in self.vehicle_names:
            self.client.armDisarm(True, drone)
        
        # Set wind speed to zero, and set a small time delay to let the drones stabilize.
        self.client.simSetWind(airsim.Vector3r(0, 0, 0))
        time.sleep(1)
        print("All drones are connected and armed.")

    def takeoff_and_hover(self):
        """Performs an asynchronous takeoff for all drones and moves them to initial hover positions."""
        # Asynchronous take-off (independent of each other).
        print("Initiating asynchronous takeoff for all drones...")
        takeoff_tasks = [self.client.takeoffAsync(vehicle_name=drone) for drone in self.vehicle_names]
        
        # Allow the drones to complete the previous command before executing the next one.
        for task in takeoff_tasks:
            task.join()
        print("Takeoff complete.")

        # Define hover position and move drones
        print("Moving to initial hover heights...")
        hover_tasks = [
            self.client.moveToPositionAsync(self.survey_area["start_x"], self.survey_area["start_y"], z, self.speed, vehicle_name=drone)
            for drone, z in zip(self.vehicle_names, self.flight_heights)
        ]
        for task in hover_tasks:
            task.join()
        print("All drones are hovering at their designated heights.")
        time.sleep(5)

    def set_camera_angles(self):
        """
        Sets the camera pitch angle for all drones.
        Yaw is handled during the flight path.
        """
        print(f"Adjusting camera pitch angle for all drones to {self.camera_pitch_angle}...")
        for drone in self.vehicle_names:
            camera_pose = airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(math.radians(self.camera_pitch_angle), 0, 0))
            self.client.simSetCameraPose("0", camera_pose, vehicle_name=drone)
        time.sleep(1) # Small delay to ensure the pose is set

    def _move_and_capture(self, x, y):
        """Helper method to move drones and capture images."""
        # Convert NumPy types to standard Python floats to avoid serialization errors
        x = float(x)
        y = float(y)
        
        move_tasks = [
            self.client.moveToPositionAsync(x, y, z, self.speed, vehicle_name=drone)
            for drone, z in zip(self.vehicle_names, self.flight_heights)
        ]
        for task in move_tasks:
            task.join()
        time.sleep(0.5)
        self.save_images(x, y)
        time.sleep(0.5)

    def fly_and_capture(self):
        """
        Flies the drones in a double-grid (cross-hatch) pattern and captures images.
        This provides improved coverage and accuracy for 3D reconstruction.
        """
        print(" - - - - - Starting Survey - - - - - ")
        self.set_camera_angles()

        x_start = self.survey_area["start_x"]
        x_end = self.survey_area["end_x"]
        y_start = self.survey_area["start_y"]
        y_end = self.survey_area["end_y"]

        # First pass: Fly back and forth along the X-axis
        print("Starting Pass 1: Flying along the X-axis...")
        current_y = y_start
        while current_y <= y_end:
            # Determine flight direction based on the row
            if int((current_y - y_start) / self.image_spacing) % 2 == 0:
                # Fly from left to right (increasing x)
                x_coords = np.arange(x_start, x_end, self.image_spacing)
            else:
                # Fly from right to left (decreasing x)
                x_coords = np.arange(x_end, x_start, -self.image_spacing)

            for current_x in x_coords:
                self._move_and_capture(current_x, current_y)
                
            current_y += self.image_spacing
        
        # Second pass: Fly back and forth along the Y-axis
        print("\nStarting Pass 2: Flying along the Y-axis (cross-hatch)...")
        current_x = x_start
        while current_x <= x_end:
            # Determine flight direction based on the column
            if int((current_x - x_start) / self.image_spacing) % 2 == 0:
                # Fly from bottom to top (increasing y)
                y_coords = np.arange(y_start, y_end, self.image_spacing)
            else:
                # Fly from top to bottom (decreasing y)
                y_coords = np.arange(y_end, y_start, -self.image_spacing)

            for current_y in y_coords:
                self._move_and_capture(current_x, current_y)

            current_x += self.image_spacing

        print("Survey path complete.")

    def save_images(self, x, y):
        """
        Captures and saves images from all drones simultaneously.

        Args:
            x (float): The current x-coordinate.
            y (float): The current y-coordinate.
        """
        print(f"Capturing images at ({x:.2f}, {y:.2f})")
        image_requests = [airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)]
        responses_list = [self.client.simGetImages(image_requests, vehicle_name=drone) for drone in self.vehicle_names]
        
        for i, (responses, drone_name) in enumerate(zip(responses_list, self.vehicle_names)):
            if responses:
                response = responses[0]
                # Convert the image to a NumPy array
                img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
                img_rgb = img1d.reshape(response.height, response.width, 3)
                
                # Create a filename with a timestamp, drone name, and position
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                filename = f"image_{drone_name}_{timestamp}_x{x:.0f}_y{y:.0f}.png"
                filepath = os.path.join(self.survey_directory, filename)
                airsim.write_png(os.path.normpath(filepath), img_rgb)
                print(f"Saved image from {drone_name} as {filepath}")

    def land_and_disarm(self):
        """Lands all drones and disarms them."""
        # Send drones to start position, disarm them and disconnect from the airsim environment.
        print("Returning to home and landing...")
        go_home_tasks = [self.client.goHomeAsync(vehicle_name=drone) for drone in self.vehicle_names]
        for task in go_home_tasks:
            task.join()
        time.sleep(2)

        print("Disarming all drones...")
        for drone in self.vehicle_names:
            self.client.armDisarm(False, drone)
            self.client.enableApiControl(False, drone)
        self.client.reset()
        print("Survey complete.")

def main():
    """Main function to run the vertical photogrammetry survey."""
    # --- CONFIGURATION ---
    # Same libraries and steps as AngledSurvey
    # Define the drone vehicles and their relative flight parameters.
    DRONE_NAMES = ["Drone6", "Drone7"]
    FLIGHT_HEIGHTS = [-25, -25] # FIXED: Same height to prevent drone visibility issues  
    CAMERA_PITCH_ANGLE = -90 # Camera pitch angle in degrees (exactly straight down)
    
    # Define the survey area
    SURVEY_AREA = {
        "start_x": 0,
        "end_x": 46,
        "start_y": 5,
        "end_y": 41
    }

    # Define flight parameters
    IMAGE_SPACING = 8 # distance between image captures (meters) - DOUBLED from 4 to 8
    DRONE_SPEED = 3 # drone speed (m/s)

    # --- SCRIPT EXECUTION ---
    survey = VerticalSurvey(
        vehicle_names=DRONE_NAMES,
        flight_heights=FLIGHT_HEIGHTS,
        survey_area=SURVEY_AREA,
        image_spacing=IMAGE_SPACING,
        speed=DRONE_SPEED,
        camera_pitch_angle=CAMERA_PITCH_ANGLE
    )

    try:
        survey.connect_and_arm()
        survey.takeoff_and_hover()
        survey.fly_and_capture()
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        survey.land_and_disarm()

if __name__ == "__main__":
    main()