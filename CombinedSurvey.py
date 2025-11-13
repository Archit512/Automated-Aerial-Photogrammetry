import threading
import time
import os
import csv
from datetime import datetime
from AngledSurvey import AngledSurvey, calculate_corners
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
        
        current_time = datetime.now()
        self.parent_survey_directory = current_time.strftime("Survey_%Y%m%d_%H%M%S")
        
        if not os.path.exists(self.parent_survey_directory):
            os.makedirs(self.parent_survey_directory)
            print(f"Created parent survey directory: {self.parent_survey_directory}")
        self.csv_filename = "survey_areas.csv"  # Default CSV filename for areas
        
    def setup_angled_survey(self):
        """Configure the angled survey parameters."""
        ANGLED_DRONE_NAMES = ["Drone5", "Drone4", "Drone3", "Drone2", "Drone1"]
        ANGLED_FLIGHT_HEIGHTS = [0, 0, 0, 0, -0.5]
        ANGLED_CAMERA_PITCH_ANGLES = [-2, -6, -9, -11, -12]

        ANGLED_SURVEY_WAYPOINTS = [
            (0, 5),    # Corner 1
            (0, 52),   # Corner 2
            (52, 52),  # Corner 3
            (52, 5)
        ]

        ANGLED_IMAGE_SPACING = 3
        ANGLED_DRONE_SPEED = 2

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
        VERTICAL_DRONE_NAMES = ["Drone6", "Drone7"]
        VERTICAL_FLIGHT_HEIGHTS = [-15, -15]
        VERTICAL_CAMERA_PITCH_ANGLE = -89
        
        # Updated bounds from diagonal points (0,52) and (-62,140)
        # Normalized to bottom-left -> top-right
        VERTICAL_SURVEY_AREA = {
            "start_x": -62,
            "end_x": 0,
            "start_y": 52,
            "end_y": 140
        }

        VERTICAL_IMAGE_SPACING = 6
        VERTICAL_DRONE_SPEED = 3
        VERTICAL_PATTERN = "grid"

        self.vertical_survey = VerticalSurvey(
            vehicle_names=VERTICAL_DRONE_NAMES,
            flight_heights=VERTICAL_FLIGHT_HEIGHTS,
            survey_area=VERTICAL_SURVEY_AREA,
            image_spacing=VERTICAL_IMAGE_SPACING,
            speed=VERTICAL_DRONE_SPEED,
            camera_pitch_angle=VERTICAL_CAMERA_PITCH_ANGLE,
            survey_type=os.path.join(self.parent_survey_directory, "VerticalSurvey"),
            pattern=VERTICAL_PATTERN  # Optional: defaults to "grid" if not specified
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
        """Run combined surveys following Angled areas from CSV (Angled-driven)."""
        self.run_combined_surveys_from_csv()

    def setup_angled_survey_for_area(self, survey_waypoints, survey_dir):
        """Setup Angled survey instance for a specific area."""
        ANGLED_DRONE_NAMES = ["Drone5", "Drone4", "Drone3", "Drone2", "Drone1"]
        ANGLED_FLIGHT_HEIGHTS = [-1, -2, -3, -4, -5]
        ANGLED_CAMERA_PITCH_ANGLES = [-2, -6, -9, -11, -12]
        ANGLED_IMAGE_SPACING = 3
        ANGLED_DRONE_SPEED = 2

        self.angled_survey = AngledSurvey(
            vehicle_names=ANGLED_DRONE_NAMES,
            flight_heights=ANGLED_FLIGHT_HEIGHTS,
            survey_waypoints=survey_waypoints,
            image_spacing=ANGLED_IMAGE_SPACING,
            speed=ANGLED_DRONE_SPEED,
            camera_pitch_angles=ANGLED_CAMERA_PITCH_ANGLES,
            survey_type=survey_dir
        )

    def setup_vertical_survey_for_area(self, survey_area, survey_dir):
        """Setup Vertical survey instance for a specific area."""
        VERTICAL_DRONE_NAMES = ["Drone6", "Drone7"]
        VERTICAL_FLIGHT_HEIGHTS = [-15, -15]
        VERTICAL_CAMERA_PITCH_ANGLE = -89
        VERTICAL_IMAGE_SPACING = 6
        VERTICAL_DRONE_SPEED = 3
        VERTICAL_PATTERN = "grid"

        self.vertical_survey = VerticalSurvey(
            vehicle_names=VERTICAL_DRONE_NAMES,
            flight_heights=VERTICAL_FLIGHT_HEIGHTS,
            survey_area=survey_area,
            image_spacing=VERTICAL_IMAGE_SPACING,
            speed=VERTICAL_DRONE_SPEED,
            camera_pitch_angle=VERTICAL_CAMERA_PITCH_ANGLE,
            survey_type=survey_dir,
            pattern=VERTICAL_PATTERN
        )

    def run_combined_surveys_from_csv(self):
        """Run Angled per area and Vertical once covering all areas."""
        print("=" * 60)
        print("COMBINED PHOTOGRAMMETRY SURVEY")
        print("=" * 60)
        print(f"Reading areas from: {self.csv_filename}")
        print("Angled: Per-area surveys | Vertical: Single survey covering all areas")
        print("=" * 60)

        # Per-run parent directory
        run_dir = os.path.join(self.parent_survey_directory, datetime.now().strftime("Run_%Y%m%d_%H%M%S"))
        os.makedirs(run_dir, exist_ok=True)

        # First pass: collect all areas and find overall bounds for Vertical
        all_areas = []
        global_min_x = float('inf')
        global_max_x = float('-inf')
        global_min_y = float('inf')
        global_max_y = float('-inf')

        try:
            with open(self.csv_filename, mode='r', encoding='utf-8-sig') as f:
                reader = csv.reader(f)
                try:
                    next(reader)  # skip header
                except StopIteration:
                    print(f"Error: CSV file '{self.csv_filename}' is empty.")
                    return

                for i, row in enumerate(reader):
                    area_index = i + 1
                    if not row or len(row) < 4:
                        print(f"Skipping malformed row {area_index}: {row}")
                        continue

                    try:
                        x1, y1 = float(row[0]), float(row[1])
                        x2, y2 = float(row[2]), float(row[3])

                        p1 = (x1, y1)
                        p2 = (x2, y2)
                        angled_waypoints = calculate_corners(p1, p2)

                        # Track global bounds
                        global_min_x = min(global_min_x, x1, x2)
                        global_max_x = max(global_max_x, x1, x2)
                        global_min_y = min(global_min_y, y1, y2)
                        global_max_y = max(global_max_y, y1, y2)

                        all_areas.append({
                            'index': area_index,
                            'p1': p1,
                            'p2': p2,
                            'waypoints': angled_waypoints
                        })

                    except ValueError:
                        print(f"Skipping row {area_index}: Invalid coordinate data. Expected 4 numbers. Got: {row}")

            if not all_areas:
                print("No valid areas found in CSV.")
                return

            # Setup single Vertical survey covering all areas
            vertical_area = {
                "start_x": global_min_x,
                "end_x": global_max_x,
                "start_y": global_min_y,
                "end_y": global_max_y,
            }
            vertical_dir = os.path.join(run_dir, "Vertical")
            os.makedirs(vertical_dir, exist_ok=True)

            print(f"\nVertical survey will cover entire region: ({global_min_x}, {global_min_y}) to ({global_max_x}, {global_max_y})")
            print(f"Total areas for Angled surveys: {len(all_areas)}\n")

            # Setup and launch Vertical survey (runs once in background)
            self.setup_vertical_survey_for_area(vertical_area, vertical_dir)
            self.vertical_thread = threading.Thread(target=self.run_vertical_survey, name="Vertical_AllAreas")
            self.vertical_thread.start()
            time.sleep(5)  # Let vertical start

            # Run Angled surveys sequentially for each area
            for area in all_areas:
                area_dir = os.path.join(run_dir, f"Area_{area['index']:03d}")
                angled_dir = os.path.join(area_dir, "Angled")
                os.makedirs(angled_dir, exist_ok=True)

                print("\n" + "-" * 60)
                print(f"STARTING ANGLED SURVEY FOR AREA {area['index']}")
                print(f"Diagonal points: {area['p1']} to {area['p2']}")
                print(f"Angled path: {area['waypoints']}")
                print("-" * 60)

                self.setup_angled_survey_for_area(area['waypoints'], angled_dir)
                self.run_angled_survey()

                print(f"Angled survey for Area {area['index']} completed.\n")

            # Wait for Vertical to complete
            print("\nWaiting for Vertical survey to complete...")
            self.vertical_thread.join()
            print("All surveys completed!")

        except FileNotFoundError:
            print(f"Error: CSV file not found at '{self.csv_filename}'")
            print("Please create 'survey_areas.csv' in the same directory.")
            print("Expected format: x1,y1,x2,y2 (one area per line)")
        except Exception as e:
            print(f"An error occurred while reading the CSV: {e}")
def main():
    """Main function to run the combined photogrammetry surveys."""
    manager = CombinedSurveyManager()
    manager.run_combined_surveys()

if __name__ == "__main__":
    main()