#!/usr/bin/env python
import rospy

# Inventory module, handles functions associated with parts inventory
class Inventory:
    # A inventory class
    def __init__(self, operators, sensors):
        # Initialise
        self.op = operators
        self.sensors = sensors
        # Build an inventory
        self.inventory = self.buildInventory()

    def buildInventory(self):
        # Build inventory using current data
        invent = {}
        # Get scan data
        while self.sensors.scanShopFloor() == False:
            rospy.sleep(0.5)
        scan = self.sensors.scanShopFloor()
        # Create a parts object
        parts = []
        # Using logical camera data get parts positions, rotations
        for x in scan['Logical Camera'].models:
            # Add part
            part = {}
            part['Type'] = x.type
            part['Pose'] = x.pose
            part['Camera pose'] = scan['Logical Camera'].pose
            part['Time'] = scan['Time']
            parts.append(part)
        # Add to inventory
        invent['Static'] = parts
        # Return
        return invent

    def updateInventory(self):
        # Update specific sections of inventory
        # For now just rebuild
        self.inventory = self.buildInventory()
        pass

    def getPart(self, part):
        # Return the pose of a part
        # Simply scan through inventory and return data for the first one found
        for x in self.inventory['Static']:
            # Check part type
            if x['Type'] == part:
                # Return part data
                return x
            else:
                # Do nothing
                pass
        # No part found
        return False

    def removePart(self, part):
        # Remove part from inventory
        self.inventory['Static'].remove(part)



        
