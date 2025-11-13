# Import all the required libraries.
import threading
import time
import os
from datetime import datetime
from AngledSurvey import AngledSurvey
from VerticalSurvey import VerticalSurvey

class CombinedSurveyManager:
    """
    A class to manage both angled and vertical surveys simultaneously.
    
    This class coordinates the execution of both survey types while ensuring
    images are saved in separate directories for each survey type.
    """
    
    def __init__(self):
        """Initialize the combined survey manager."""
        self.angled_survey = None
        self.vertical_survey = None
        self.angled_thread = None
        self.vertical_thread = None
        self.survey_results = {"angled": None, "vertical": None}
        
        # Create parent survey directory with timestamp
        current_time = datetime.now()
        self.parent_survey_directory = current_time.strftime("Survey_%Y%m%d_%H%M%S")
        
        # Create the parent directory if it doesn't exist
        if not os.path.exists(self.parent_survey_directory):
            os.makedirs(self.parent_survey_directory)
            print(f"Created parent survey directory: {self.parent_survey_directory}")
        
    def setup_angled_survey(self):
        """Configure the angled survey parameters."""
        # Define the angled survey configuration
        ANGLED_DRONE_NAMES = ["Drone5", "Drone4", "Drone3", "Drone2", "Drone1"]
        ANGLED_FLIGHT_HEIGHTS = [0, 0, 0, 0, -0.5] # Negative Z values for height above ground
        ANGLED_CAMERA_PITCH_ANGLES = [-2, -6, -9, -11, -12]  # Pitch angles in degrees

        # Define the survey area using four corners (x, y)
        ANGLED_SURVEY_WAYPOINTS = [
            (0, 5),    # Corner 1
            (0, 52),   # Corner 2
            (52, 52),  # Corner 3
            (52, 5)    # Corner 4
        ]

        # Define flight parameters
        ANGLED_IMAGE_SPACING = 3  # distance between image captures (meters)
        ANGLED_DRONE_SPEED = 2    # drone speed (m/s)

        self.angled_survey = AngledSurvey(
            vehicle_names=ANGLED_DRONE_NAMES,
            flight_heights=ANGLED_FLIGHT_HEIGHTS,
            survey_waypoints=ANGLED_SURVEY_WAYPOINTS,
            image_spacing=ANGLED_IMAGE_SPACING,
            speed=ANGLED_DRONE_SPEED,
            camera_pitch_angles=ANGLED_CAMERA_PITCH_ANGLES,
            survey_type=os.path.join(self.parent_survey_directory, "AngledSurvey")
        )
        
    def setup_vertical_survey(self):
        """Configure the vertical survey parameters."""
        # Define the vertical survey configuration
        VERTICAL_DRONE_NAMES = ["Drone6", "Drone7"]
        VERTICAL_FLIGHT_HEIGHTS = [-15, -15]  # FIXED: Same height to prevent drone visibility issues
        VERTICAL_CAMERA_PITCH_ANGLE = -89     # Camera pitch angle in degrees (exactly straight down)
        
        # Define the survey area
        VERTICAL_SURVEY_AREA = {
            "start_x": 0,
            "end_x": 46,
            "start_y": 5,
            "end_y": 41
        }

        # Define flight parameters
        VERTICAL_IMAGE_SPACING = 8  # distance between image captures (meters) - DOUBLED from 4 to 8
        VERTICAL_DRONE_SPEED = 3    # drone speed (m/s)

        self.vertical_survey = VerticalSurvey(
            vehicle_names=VERTICAL_DRONE_NAMES,
            flight_heights=VERTICAL_FLIGHT_HEIGHTS,
            survey_area=VERTICAL_SURVEY_AREA,
            image_spacing=VERTICAL_IMAGE_SPACING,
            speed=VERTICAL_DRONE_SPEED,
            camera_pitch_angle=VERTICAL_CAMERA_PITCH_ANGLE,
            survey_type=os.path.join(self.parent_survey_directory, "VerticalSurvey")
        )
    
    def run_angled_survey(self):
        """Run the angled survey in a separate thread."""
        try:
            print("\n[ANGLED SURVEY] Starting angled survey...")
            self.angled_survey.connect_and_arm()
            self.angled_survey.takeoff_and_hover()
            self.angled_survey.fly_and_capture()
            self.survey_results["angled"] = "SUCCESS"
            print("\n[ANGLED SURVEY] Angled survey completed successfully!")
        except Exception as e:
            print(f"\n[ANGLED SURVEY] An error occurred in angled survey: {e}")
            self.survey_results["angled"] = f"ERROR: {e}"
        finally:
            if self.angled_survey:
                self.angled_survey.land_and_disarm()
    
    def run_vertical_survey(self):
        """Run the vertical survey in a separate thread."""
        try:
            print("\n[VERTICAL SURVEY] Starting vertical survey...")
            self.vertical_survey.connect_and_arm()
            self.vertical_survey.takeoff_and_hover()
            self.vertical_survey.fly_and_capture()
            self.survey_results["vertical"] = "SUCCESS"
            print("\n[VERTICAL SURVEY] Vertical survey completed successfully!")
        except Exception as e:
            print(f"\n[VERTICAL SURVEY] An error occurred in vertical survey: {e}")
            self.survey_results["vertical"] = f"ERROR: {e}"
        finally:
            if self.vertical_survey:
                self.vertical_survey.land_and_disarm()
    
    def run_combined_surveys(self):
        """Run both surveys simultaneously."""
        print("=" * 60)
        print("COMBINED PHOTOGRAMMETRY SURVEY")
        print("=" * 60)
        print("Starting both Angled and Vertical surveys simultaneously...")
        print("Images will be saved in separate directories for each survey type.")
        print("=" * 60)
        
        # Setup both surveys
        self.setup_angled_survey()
        self.setup_vertical_survey()
        
        # Create threads for both surveys
        self.angled_thread = threading.Thread(target=self.run_angled_survey, name="AngledSurveyThread")
        self.vertical_thread = threading.Thread(target=self.run_vertical_survey, name="VerticalSurveyThread")
        
        # Start both surveys simultaneously
        print("\nLaunching both survey threads...")
        self.angled_thread.start()
        time.sleep(10)  # Small delay to stagger the start
        self.vertical_thread.start()
        time.sleep(10)
        print("\nBoth surveys are now in progress...")
        # Wait for both surveys to complete
        print("\nWaiting for surveys to complete...")
        self.angled_thread.join()
        self.vertical_thread.join()
        
        # Print final results
        print("\n" + "=" * 60)
        print("SURVEY COMPLETION REPORT")
        print("=" * 60)
        print(f"Angled Survey Status: {self.survey_results['angled']}")
        print(f"Vertical Survey Status: {self.survey_results['vertical']}")
        
        if self.angled_survey and hasattr(self.angled_survey, 'survey_directory'):
            print(f"Angled Survey Images Directory: {self.angled_survey.survey_directory}")
        if self.vertical_survey and hasattr(self.vertical_survey, 'survey_directory'):
            print(f"Vertical Survey Images Directory: {self.vertical_survey.survey_directory}")
        
        print("=" * 60)
        print("Combined survey operation completed.")

def main():
    """Main function to run the combined photogrammetry surveys."""
    manager = CombinedSurveyManager()
    manager.run_combined_surveys()

if __name__ == "__main__":
    main()