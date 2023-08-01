from dronekit import connect

# Connect to the Vehicle (in this case a UDP endpoint)
vehicle = connect('/dev/serial0', wait_ready=True, baud=921600)

from dronekit import connect, VehicleMode
# Get some vehicle attributes (state)
print ("Get some vehicle attribute values:\n")

print (" GPS: %s\n" % vehicle.gps_0)

print (" Battery: %s\n" % vehicle.battery)

print (" Last Heartbeat: %s\n" % vehicle.last_heartbeat)

print (" Is Armable?: %s\n" % vehicle.is_armable)

print (" System status: %s\n" % vehicle.system_status.state)

print (" Mode: %s\n" % vehicle.mode.name)    # settable

# Close vehicle object before exiting script
vehicle.close()
