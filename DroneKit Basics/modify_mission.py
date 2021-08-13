# Connect to the Vehicle (in this case a simulated vehicle at 127.0.0.1:14550)
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Get the set of commands from the vehicle
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

# Save the vehicle commands to a list
missionlist=[]
for cmd in cmds:
    missionlist.append(cmd)

# Modify the mission as needed. For example, here we change the
# first waypoint into a MAV_CMD_NAV_TAKEOFF command.
missionlist[0].command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF

# Clear the current mission (command is sent when we call upload())
cmds.clear()

#Write the modified mission and flush to the vehicle
for cmd in missionlist:
    cmds.add(cmd)
cmds.upload()