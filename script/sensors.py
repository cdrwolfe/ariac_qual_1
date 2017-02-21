#!/usr/bin/env python
import rospy

# Sensors module, handles functions associated with sensor data, objects
class Sensors:
    # A sensor class
    def __init__(self, operators):
        # Initialise
        self.op = operators
        # Build an inventory of all parts under view       

    def scanShopFloor(self):
        # Using available sensors acquire psrt data (position, rotation, etc)
        logCameraParts = self.op.logicalCameraData
        # Laser profiler
        laserProfilerParts = self.op.laserProfilerData
        # Break beam sensor
        breakBeamParts = self.op.breakBeamData
        # Proximty sensor
        proximityParts = self.op.proximitySensorData
        # Set time stamp
        time = rospy.get_time()
        # Create object to return
        scan = {}
        scan['Logical Camera'] = logCameraParts
        scan['Laser Profiler'] = laserProfilerParts
        scan['Break Beam'] = breakBeamParts
        scan['Time'] = time
        # Check data exists
        if scan['Logical Camera'] != None and scan['Laser Profiler'] != None: 
            # Return
            return scan
        else:
            # Return
            return False

