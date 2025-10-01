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
    This version uses an interleaved flight path to divide work between drones
    and prevent them from appearing in each other's images.
    """

    def __init__(self, vehicle_names, flight_heights, survey_area, image_spacing, speed, camera_pitch_angle, survey_type="VerticalSurvey"):
        """
        Initializes the survey object with required parameters.
        """
        self.client = airsim.MultirotorClient()
        self.vehicle_names = vehicle_names
        self.flight_heights = flight_heights
        self.survey_area = survey_area
        self.image_spacing = image_spacing
        self.speed = speed
        self.camera_pitch_angle = camera_pitch_angle
        
        current_time = datetime.now()
        if os.path.sep in survey_type:
            self.survey_directory = survey_type
        else:
            self.survey_directory = current_time.strftime(f"{survey_type}_%Y%m%d_%H%M%S")
        
        if not os.path.exists(self.survey_directory):
            os.makedirs(self.survey_directory)
            print(f"Created survey directory: {self.survey_directory}")

    def connect_and_arm(self):
        """Connects to AirSim and arms all drones."""
        print("Connecting to AirSim...")
        self.client.confirmConnection()
        for drone in self.vehicle_names:
            self.client.enableApiControl(True, drone)
            print(f"Enabling API control for {drone}...")
        
        print("Arming all drones...")
        for drone in self.vehicle_names:
            self.client.armDisarm(True, drone)
        
        self.client.simSetWind(airsim.Vector3r(0, 0, 0))
        time.sleep(1)
        print("All drones are connected and armed.")

    def takeoff_and_hover(self):
        """Performs an asynchronous takeoff for all drones and moves them to initial hover positions."""
        print("Initiating asynchronous takeoff for all drones...")
        takeoff_tasks = [self.client.takeoffAsync(vehicle_name=drone) for drone in self.vehicle_names]
        
        for task in takeoff_tasks:
            task.join()
        print("Takeoff complete.")

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
        """Sets the camera pitch angle for all drones."""
        print(f"Adjusting camera pitch angle for all drones to {self.camera_pitch_angle}...")
        for drone in self.vehicle_names:
            camera_pose = airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(math.radians(self.camera_pitch_angle), 0, 0))
            self.client.simSetCameraPose("0", camera_pose, vehicle_name=drone)
        time.sleep(1)

    def _fly_path_and_capture(self, vehicle_name, z_height, path):
        """
        Helper method to fly a single drone along a specific path (a list of (x,y) tuples)
        and capture images at each point.
        """
        print(f"--- {vehicle_name} is starting its assigned path. ---")
        for x, y in path:
            # Move a single drone to its target point
            move_task = self.client.moveToPositionAsync(float(x), float(y), z_height, self.speed, vehicle_name=vehicle_name)
            move_task.join()
            time.sleep(0.5) # Allow drone to stabilize
            
            # Capture and save an image from that one drone
            # print(f"Capturing image from {vehicle_name} at ({x:.2f}, {y:.2f})")
            image_requests = [airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)]
            responses = self.client.simGetImages(image_requests, vehicle_name=vehicle_name)
            
            if responses and responses[0]:
                response = responses[0]
                img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
                img_rgb = img1d.reshape(response.height, response.width, 3)
                
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                filename = f"image_{vehicle_name}_{timestamp}_x{x:.0f}_y{y:.0f}.png"
                filepath = os.path.join(self.survey_directory, filename)
                airsim.write_png(os.path.normpath(filepath), img_rgb)
        
        print(f"--- {vehicle_name} has completed its path. ---")

    def fly_and_capture(self):
        """
        Flies drones in interleaved parallel paths for a cross-hatch pattern
        to cover the area efficiently and avoid frame obstruction. Assumes two drones.
        """
        print(" - - - - - Starting Interleaved Survey - - - - - ")
        self.set_camera_angles()

        if len(self.vehicle_names) != 2:
            print("Error: Interleaved flight path is designed for exactly two drones.")
            return

        drone1_name, drone2_name = self.vehicle_names[0], self.vehicle_names[1]
        drone1_z, drone2_z = self.flight_heights[0], self.flight_heights[1]

        x_start = self.survey_area["start_x"]
        x_end = self.survey_area["end_x"]
        y_start = self.survey_area["start_y"]
        y_end = self.survey_area["end_y"]

        # --- First pass: Fly along X-axis (dividing Y-rows) ---
        print("Starting Pass 1: Interleaved flight along X-axis...")
        y_coords = np.arange(y_start, y_end + 1, self.image_spacing)
        
        for i, current_y in enumerate(y_coords):
            # Determine flight direction based on the overall row index for snake pattern
            if i % 2 == 0:
                x_path = np.arange(x_start, x_end + 1, self.image_spacing)
            else:
                x_path = np.arange(x_end, x_start - 1, -self.image_spacing)
            
            path_tuples = [(x, current_y) for x in x_path]

            # Assign this row to one of the drones
            if i % 2 == 0: # Drone 1 takes even rows (0, 2, 4...)
                self._fly_path_and_capture(drone1_name, drone1_z, path_tuples)
            else: # Drone 2 takes odd rows (1, 3, 5...)
                self._fly_path_and_capture(drone2_name, drone2_z, path_tuples)
        
        # --- Second pass: Fly along Y-axis (dividing X-columns) ---
        print("\nStarting Pass 2: Interleaved flight along Y-axis...")
        x_coords = np.arange(x_start, x_end + 1, self.image_spacing)

        for i, current_x in enumerate(x_coords):
            # Determine flight direction for snake pattern
            if i % 2 == 0:
                y_path = np.arange(y_start, y_end + 1, self.image_spacing)
            else:
                y_path = np.arange(y_end, y_start - 1, -self.image_spacing)

            path_tuples = [(current_x, y) for y in y_path]

            # Assign this column to one of the drones
            if i % 2 == 0: # Drone 1 takes even columns
                self._fly_path_and_capture(drone1_name, drone1_z, path_tuples)
            else: # Drone 2 takes odd columns
                self._fly_path_and_capture(drone2_name, drone2_z, path_tuples)

        print("Survey path complete.")

    def land_and_disarm(self):
        """Lands all drones and disarms them."""
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
    DRONE_NAMES = ["Drone6", "Drone7"]
    
    # IMPORTANT: With interleaved paths, you can set the same height for consistent results!
    FLIGHT_HEIGHTS = [-25, -25] # Both drones at the same altitude.
    CAMERA_PITCH_ANGLE = -90 # Exactly nadir (straight down)
    
    # Define the survey area
    SURVEY_AREA = {
        "start_x": 0,
        "end_x": 46,
        "start_y": 0,
        "end_y": 42
    }

    # Define flight parameters
    IMAGE_SPACING = 4 # distance between image captures (meters)
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