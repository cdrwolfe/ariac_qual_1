#!/usr/bin/env python
import rospy
import operators
import orders
import sensors
import inventory
import armController
from std_srvs.srv import Trigger


# Start the competition by waiting for and then calling the start ROS Service.
def Start():
    # Wait for service to be active
    rospy.loginfo("Waiting for the competition to be ready...")
    rospy.wait_for_service('/ariac/start_competition')
    start_service = rospy.ServiceProxy('/ariac/start_competition', Trigger)
    # Success
    rospy.loginfo("Competition is now ready.")
    rospy.loginfo("Requesting competition start...")
    # Call start
    resp = start_service()
    # Check if successful
    if (resp.success != True):
        rospy.logerr("Failed to start the competition: " + resp.message);
    else:
        rospy.loginfo("Competition started!")

def FloorShopManager():
    #  Declare variables
    op = operators.Operators()
    sen = sensors.Sensors(op)
    arm = armController.ArmController(op)
    inv = inventory.Inventory(op, sen) 
    # Loop through process of completing orders on the shop floor
    while not rospy.is_shutdown():
        # Listen for orders
        if len(op.currentOrders) > 0:
            # There are orders to be processed
            order = orders.Orders(op.currentOrders[0])
            agv = 'agv1'
            tray = 'Left'
            # loop through each kit and process
            for kit in order.kits:
                # Get type of parts we need to add to tray
                for part in kit['Objects']:
                    # Set procede to false
                    proceed = False
                    # Check whether this part has successfully being placed on tray and should procede
                    while proceed == False:
                        # Return part location from inventory
                        inventoryPart = False
                        # Set desired pose of part in tray
                        desiredPose = part.pose
                        # Wait for step to complete
                        while inventoryPart == False:
                            # Loop and wait for part to be found
                            inventoryPart = inv.getPart(part.type)
                            # Get part pose
                            rospy.loginfo("Part: " + str(inventoryPart))
                            rospy.loginfo("Desired Pose: " + str(desiredPose))
                            # Check if found, if not wait and try again
                            if inventoryPart == False:
                                # Wait
                                rospy.sleep(1.0)
                                # Scan inventory and update (this is really only for later when new parts
                                # are being actively placed on belt)
                                inv.updateInventory()
                        print "=========== Transfer Part: " + str(inventoryPart['Type'])
                        while op.gripperStateData.attached != True:
                            # Move arm to above part position
                            pose = arm.transformPose(inventoryPart['Pose'], [-0.2,0,0], [0,0,0,0], 'logical_camera_1_frame')
                            arm.poseTarget(pose)
                            rospy.loginfo("Arm Pose for part" + str(pose))
                            # Plan
                            arm.planPose()
                            # Execute plan
                            arm.executePlan()
                            # Wait
                            rospy.sleep(1.5)
                            # Move arm into contact with part
                            pose = arm.transformPose(inventoryPart['Pose'], [-0.02,0,0], [0,0,0,0], 'logical_camera_1_frame')
                            arm.poseTarget(pose)
                            # Plan
                            arm.planPose()
                            # Execute plan
                            arm.executePlan()
                            # Wait
                            rospy.sleep(0.5)
                            # Activate gripper first attempt
                            arm.gripper(True)
                            # Check
                            if op.gripperStateData.attached != True:
                                # Gripper failed
                                print "Gripper attempt failed!"
                                # Move arm into contact with part
                                pose = arm.transformPose(inventoryPart['Pose'], [-0.01,0,0], [0,0,0,0], 'logical_camera_1_frame')
                                arm.poseTarget(pose)
                                arm.planPose()
                                #raw_input("Press Enter to continue...")
                                arm.executePlan()
                                # Wait
                                rospy.sleep(0.5)
                                # Set count
                                count = 0
                                # Activate gripper in loop for 1.0 second
                                while op.gripperStateData.attached != True and count < 5:
                                    arm.gripper(True)
                                    # Update count and wait
                                    count += 1
                                    # Wait
                                    rospy.sleep(0.2)
                            while arm.armState() == 'Moving':
                                # Hold here
                                pass
                        #raw_input("Press Enter to continue...")
                        # Wait
                        rospy.sleep(1.0)
                        while arm.armState() == 'Moving':
                            # Hold here
                            pass
                        # Return arm to holding position
                        arm.handlePart()
                        # Wait 
                        rospy.sleep(1.5)
                        while arm.armState() == 'Moving':
                            # Hold here
                            pass
                        #raw_input("Press Enter to continue...")
                        # Move part to desired tray location
                        arm.moveToTray(tray)
                        #raw_input("Press Enter to continue...")
                        # Wait 
                        rospy.sleep(0.5)
                        while arm.armState() == 'Moving':
                            # Hold here
                            pass
                        # Move part to desired tray location
                        arm.moveOverTray(tray)
                        #raw_input("Press Enter to continue...")
                        # Wait 
                        rospy.sleep(0.5)
                        while arm.armState() == 'Moving':
                            # Hold here
                            pass
                        # Get tray pose
                        trayPose = arm.trayPose(agv + '_load_point_frame')
                        if agv == 'agv1':
                            # Check desired pose values
                            if desiredPose.position.x > 0.12:
                                # Lower value
                                desiredPose.position.x = 0.1
                            if desiredPose.position.y > 0.12:
                                # Lower value
                                desiredPose.position.y = 0.055
                            if desiredPose.position.x < -0.12:
                                # Lower value
                                desiredPose.position.x = -0.1
                        elif agv == 'agv2':
                            # Check desired pose values
                            if desiredPose.position.x > 0.09:
                                # Lower value
                                desiredPose.position.x = 0.055
                            if desiredPose.position.y < -0.12:
                                # Lower value
                                desiredPose.position.y = -0.055
                            if desiredPose.position.x < -0.12:
                                # Lower value
                                desiredPose.position.x = -0.1
                        # Offset to desired pose of part on tray
                        trayPose.pose.position.x += desiredPose.position.x
                        trayPose.pose.position.y += desiredPose.position.y
                        trayPose.pose.position.z += (desiredPose.position.z + 0.3)
                        # Set orinetation, currently just use these values, not sure how to get correct orientation from world yet
                        trayPose.pose.orientation.x = 0 
                        trayPose.pose.orientation.y = 0.917 
                        trayPose.pose.orientation.z = 0
                        trayPose.pose.orientation.w = 0.398
                        # Place part in correct position on tray according to order
                        arm.poseTarget(trayPose)
                        arm.planPose()
                        arm.executePlan()
                        # Wait 
                        rospy.sleep(0.5)
                        while arm.armState() == 'Moving':
                            # Hold here
                            pass
                        # Check whether we should procede
                        if op.gripperStateData.attached != True:
                            # Part has fallen off
                            print "Gripper is empty, Oh My!!"
                            proceed = False
                        else:
                            # Successful
                            proceed = True
                        # Inactivate gripper
                        arm.gripper(False)
                        # Wait 
                        rospy.sleep(0.5)
                        while arm.armState() == 'Moving':
                            # Hold here
                            pass
                        # Remove part from inventory
                        inv.removePart(inventoryPart)
                        # Retract arm
                        arm.moveToTray(tray)
                        # Wait 
                        rospy.sleep(0.5)
                        while arm.armState() == 'Moving':
                            # Hold here
                            pass
                        # Return arm to holding position
                        arm.handlePart()
                        # Wait 
                        rospy.sleep(0.5)
                        while arm.armState() == 'Moving':
                            # Hold here
                            pass
                        # Move over inventory
                        arm.moveOverInventory()
                        # Wait for some reason otherwise moveit screws up
                        rospy.sleep(1.0)
                        # Update inventory
                        inv.buildInventory()
                        
                        
            # Send AGV away
            print "Sending AGV away,..."
            if agv == 'agv1':
                # Submit 1st agv
                op.submitAGV(0, 'order_0')
            else:
                 # Submit 2nd agv
                op.submitAGV(1, 'order_0')
        else:
            # Do nothing and wait
            pass



 

if __name__ == '__main__':
    try:
        rospy.init_node('ariac_qual_1')
        # Start competition
        Start()
        # Call floor shop manager
        FloorShopManager()
        # Spin up ROS
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
