import airsim

client = airsim.MultirotorClient()
client.confirmConnection()
print("Connected to AirSim!")

client.enableApiControl(True)
client.armDisarm(True)
print("API control enabled and drone armed.")

client.takeoffAsync().join()
print("Drone has taken off.")


vx = 5  
vy = 0  
vz = 0  
duration = 3 
client.moveByVelocityAsync(vx, vy, vz, duration).join()
print("Drone moved forward.")

# Move upward
client.moveByVelocityAsync(0, 0, -5, 3).join()
print("Drone moved upward.")

# Move forward
client.moveByVelocityAsync(5, 0, 0, 3).join()
print("Drone moved forward.")