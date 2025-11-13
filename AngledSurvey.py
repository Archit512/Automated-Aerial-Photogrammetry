import airsim # type: ignore
import time
import os
import numpy as np # type is ignore
import math
from datetime import datetime

class AngledSurvey:
    """
    A class to handle a multi-drone photogrammetry survey in AirSim.

    This class provides a structured and efficient way to control multiple drones,
    fly a predefined path, and capture images for photogrammetry.
    """

    def __init__(self, vehicle_names, flight_heights, survey_waypoints, image_spacing, speed, camera_pitch_angles, survey_type="AngledSurvey"):
        """
        Initializes the survey object with required parameters.

        Args:
            vehicle_names (list): A list of vehicle names (e.g., ["Drone1", "Drone2"]).
            flight_heights (list): A list of flight heights (negative Z values) for each drone.
            survey_waypoints (list): A list of (x, y) coordinates for the survey path corners.
            image_spacing (float): The distance in meters between each image capture point.
            speed (float): The speed of the drones in m/s.
            camera_pitch_angles (list): The pitch angles (in degrees) for each drone's camera.
            survey_type (str): Type of survey for directory naming (default: "AngledSurvey").
        """
        self.client = airsim.MultirotorClient()
        self.vehicle_names = vehicle_names
        self.flight_heights = flight_heights
        self.survey_waypoints = survey_waypoints
        self.image_spacing = image_spacing
        self.speed = speed
        self.camera_pitch_angles = camera_pitch_angles
        
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
            self.client.moveToPositionAsync(0, 0, z, self.speed, vehicle_name=drone, lookahead=-1)
            for drone, z in zip(self.vehicle_names, self.flight_heights)
        ]
        for task in hover_tasks:
            task.join()
        print("All drones are hovering at their designated heights.")

        
    def set_camera_angles(self, yaw_angle):
        """
        Sets the camera pitch and yaw for all drones.

        Args:
            yaw_angle (float): The yaw angle in degrees to face the target.
        """
        print(f"Adjusting camera angles for all drones to face yaw {yaw_angle}...")
        # Move camera slightly forward (e.g., 1.00 meters along X-axis)
        offset_distance = 2.0
        yaw_rad = math.radians(yaw_angle)
        offset_x = offset_distance * math.cos(yaw_rad)
        offset_y = offset_distance * math.sin(yaw_rad)
        camera_offset = airsim.Vector3r(offset_x, offset_y, -0.2)   

        for drone, pitch in zip(self.vehicle_names, self.camera_pitch_angles):
            # Use a less negative pitch if needed (e.g., max(-10, pitch))
            safe_pitch = max(-10, pitch)
            camera_pose = airsim.Pose(camera_offset, airsim.to_quaternion(math.radians(safe_pitch), 0, math.radians(yaw_angle)))
            self.client.simSetCameraPose("0", camera_pose, vehicle_name=drone)
        time.sleep(1)

    def fly_and_capture(self):
            """
            Flies the drones along the survey path and captures images at intervals.
            """
            # Define camera orientation, drone path, image generation and file path for each drone.
            print(" - - - - - Starting Survey - - - - - ")
            num_waypoints = len(self.survey_waypoints)
            if num_waypoints < 2:
                print("Not enough waypoints to create a path.")
                return

            for i in range(num_waypoints):
                start_x, start_y = self.survey_waypoints[i]
                end_x, end_y = self.survey_waypoints[(i + 1) % num_waypoints]

                # Calculate the direction and number of steps
                dx = end_x - start_x
                dy = end_y - start_y
                distance = math.sqrt(dx**2 + dy**2)
                num_steps = int(distance / self.image_spacing)
                yaw_angle = math.degrees(math.atan2(dy, dx)) - 90  # Adjust for camera orientation
                print(f"Flying from ({start_x}, {start_y}) to ({end_x}, {end_y}) with {num_steps} steps...")

                self.set_camera_angles(yaw_angle)

                for step in range(num_steps + 1):
                    # Calculate the position for this step
                    interp_x = start_x + (dx * step / num_steps)
                    interp_y = start_y + (dy * step / num_steps)

                    # Move all drones to the new position
                    move_tasks = [
                        self.client.moveToPositionAsync(interp_x, interp_y, z, self.speed, vehicle_name=drone)
                        for drone, z in zip(self.vehicle_names, self.flight_heights)
                    ]
                    for task in move_tasks:
                        task.join()
                    time.sleep(1.5) # Give more time for the drone to stabilize at the new position

                    print(f"Capturing images at ({interp_x:.2f}, {interp_y:.2f})")
                    self.save_images(interp_x, interp_y)
                    time.sleep(1.0) # Give more time for the capture to complete

            print("Survey path complete.")



    def save_images(self, x, y):
        """
        Captures and saves images from all drones simultaneously.

        Args:
            x (float): The current x-coordinate.
            y (float): The current y-coordinate.
        """
        image_requests = [airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)]
        responses_list = [self.client.simGetImages(image_requests, vehicle_name=drone) for drone in self.vehicle_names]
        
        for i, (responses, drone_name) in enumerate(zip(responses_list, self.vehicle_names)):
            if responses:
                response =  responses[0]
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
    """Main function to run the photogrammetry survey."""
    # --- CONFIGURATION ---
    # Define the drone vehicles and their relative flight parameters.
    DRONE_NAMES = ["Drone5", "Drone4", "Drone3", "Drone2", "Drone1"]
    FLIGHT_HEIGHTS  = [-1, -2, -3, -4, -5] # Negative Z values for height above ground
    CAMERA_PITCH_ANGLES = [-2, -6, -9, -11, -12] # Pitch angles in degrees

    # Define the survey area using four corners (x, y)based in sett
    # The order of waypoints defines the flight path (e.g., clockwise or counter-clockwise)
    SURVEY_WAYPOINTS = [
        (0, 5),    # Corner 1
        (0, 52),   # Corner 2
        (52, 52),  # Corner 3
        (52, 5)    # Corner 4
    ]

    # Define flight parameters
    IMAGE_SPACING = 3 # distance between image captures (meters)
    DRONE_SPEED = 2 # drone speed (m/s)

    # --- SCRIPT EXECUTION ---
    survey = AngledSurvey(
        vehicle_names=DRONE_NAMES,
        flight_heights=FLIGHT_HEIGHTS,
        survey_waypoints=SURVEY_WAYPOINTS,
        image_spacing=IMAGE_SPACING,
        speed=DRONE_SPEED,
        camera_pitch_angles=CAMERA_PITCH_ANGLES
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