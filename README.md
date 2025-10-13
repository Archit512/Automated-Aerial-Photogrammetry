# Automated Aerial Photogrammetry

## Project Overview
Automated Aerial Photogrammetry is a project designed to perform multi-drone photogrammetry surveys using AirSim. The project includes two types of surveys:
- **Angled Survey**: Captures images at an angle for detailed 3D reconstruction.
- **Vertical Survey**: Captures top-down images for mapping and orthophoto generation.

The project is implemented in Python and leverages AirSim's API to control drones, capture images, and save them in structured directories.

---

## Features
- Multi-drone support for simultaneous surveys.
- Configurable flight parameters such as speed, height, and image spacing.
- Separate directories for angled and vertical surveys.
- Threaded execution for running multiple surveys concurrently.

---

## Prerequisites
1. **AirSim**: Ensure AirSim is installed and running. You can download it from the [AirSim GitHub repository](https://github.com/microsoft/AirSim).
2. **Python**: Install Python 3.8 or later.
3. **Dependencies**: Install the required Python packages using the following command:
   ```bash
   pip install -r requirements.txt
   ```

---

## Setting Up AirSimNH
AirSimNH is a pre-configured environment for running the photogrammetry surveys. Follow these steps to set it up:

1. **Download AirSimNH**:
   - Clone or download the AirSimNH environment.

2. **Launch AirSimNH**:
   - Navigate to the `WindowsNoEditor` folder.
   - Run the `AirSimNH.exe` file to launch the environment.

3. **Verify Connection**:
   - Use the `test_connection.py` script to ensure the Python client can connect to AirSim.
   - Run the script:
     ```bash
     python test_connection.py
     ```

---

## Changing `settings.json`
The `settings.json` file in the AirSim directory configures the simulation environment. To modify it:

1. **Locate the File**:
   - The file is typically located in `Documents\AirSim\settings.json`.

2. **Edit the File**:
   - Open the file in a text editor.
   - Example configuration:
     ```json
     {
       "SettingsVersion": 1.2,
       "SimMode": "Multirotor",
       "Vehicles": {
         "Drone1": {
           "VehicleType": "SimpleFlight"
         },
         "Drone2": {
           "VehicleType": "SimpleFlight"
         }
       }
     }
     ```

3. **Save and Restart**:
   - Save the changes and restart AirSimNH to apply the new settings.

### Editing or Copying `settings.json`

A `settings.json` file has been included in this folder. To use it:

1. **Edit the File**:
   - Open the `settings.json` file in this folder.
   - Modify the configuration as needed (e.g., add or adjust vehicles).

2. **Copy the File**:
   - Copy the `settings.json` file to the `Documents\AirSim` directory.

3. **Restart AirSim**:
   - Restart the AirSim simulation to apply the new settings.

---

### Note on Editing `settings.json`

If you edit the `settings.json` file to add or modify vehicles, you may need to update the code in the scripts to reflect these changes. For example:

1. **Update Vehicle Names**:
   - Ensure the `vehicle_names` list in the scripts matches the names of the vehicles in `settings.json`.

2. **Adjust Parameters**:
   - Update flight heights, camera angles, or other parameters to align with the new vehicle configurations.

3. **Test the Changes**:
   - Run the `test_connection.py` script to verify that the updated vehicles can connect to AirSim.

Always test your changes to ensure compatibility between the `settings.json` file and the scripts.

---

## Running the Surveys
1. **Angled Survey**:
   - Run the `AngledSurvey.py` script to perform an angled survey.
   ```bash
   python AngledSurvey.py
   ```

2. **Vertical Survey**:
   - Run the `VerticalSurvey.py` script to perform a vertical survey.
   ```bash
   python VerticalSurvey.py
   ```

3. **Combined Survey**:
   - Run the `CombinedSurvey.py` script to perform both surveys simultaneously.
   ```bash
   python CombinedSurvey.py
   ```

---

## Directory Structure
- **AngledSurvey**: Contains images and logs for angled surveys.
- **VerticalSurvey**: Contains images and logs for vertical surveys.
- **WindowsNoEditor**: Contains the AirSimNH environment.

---

## Troubleshooting
- **Connection Issues**:
  - Ensure AirSimNH is running.
  - Verify the `settings.json` file is correctly configured.
- **Missing Dependencies**:
  - Run `pip install -r requirements.txt` to install missing packages.
- **Simulation Crashes**:
  - Check the AirSim logs in the `Saved\Logs` directory.

---

## License
This project is licensed under the MIT License. See the LICENSE file for details.