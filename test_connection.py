import airsim
import time

def test_airsim_connection():
    """Test AirSim connection with timeout."""
    print("Testing AirSim connection...")
    
    try:
        # Create client with timeout
        client = airsim.MultirotorClient()
        
        # Set a timeout for connection
        print("Attempting to confirm connection...")
        
        # Try to connect with a timeout mechanism
        start_time = time.time()
        timeout = 30  # 30 seconds timeout
        
        while time.time() - start_time < timeout:
            try:
                client.confirmConnection()
                print("✓ Successfully connected to AirSim!")
                
                # Get simulation state
                print("Getting simulation state...")
                sim_state = client.getMultirotorState()
                print(f"✓ Simulation state retrieved: {sim_state.collision}")
                
                return True
                
            except Exception as e:
                print(f"Connection attempt failed: {e}")
                time.sleep(2)  # Wait 2 seconds before retry
                
        print("✗ Connection timeout after 30 seconds")
        return False
        
    except Exception as e:
        print(f"✗ Failed to create AirSim client: {e}")
        return False

if __name__ == "__main__":
    test_airsim_connection()