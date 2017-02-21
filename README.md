# ARIAC Qualification Task 1
Agile Robotics for Industrial Automation Competition

The objective of the Agile Robotics for Industrial Automation Competition (ARIAC) is to test the agility of industrial robot systems, with the goal of enabling industrial robots on the shop floors to be more productive, more autonomous, and to require less time from shop floor workers. 

Details:

http://gazebosim.org/ariac

The actual competition is to take place on June 2017, however in order to qualify you first need to complete 1 of the 3 upcoming qualifier tasks, with each one becoming more complicated as you progress.

I have bashed together a rather crude example on how to complete the first qualification task, which is essentially a simplified version of what you will face in the real competiton, a mix of pick and place, and how to react when a part is dropped. further details can be found here: https://bitbucket.org/osrf/ariac/wiki/qualifiers/qual1

To get this up and running, first follow the installation instructions found here:

http://wiki.ros.org/ariac/Tutorials/SystemSetup

Also be sure to have MoveIt! installed and the UR10 package, installation instructions for which can be found here:

http://wiki.ros.org/ariac/Tutorials/MoveItInterface

I use my normal catkin_ws in this case and not ariac_ws.

Feel free to have a look at the other tutorials to get an idea of how it should run, my solution is by no means complete or efficient.

Download the ariac_qual_1 package into you catkin workspace, i.e 'catkin_ws/src' and then run catkin_make.

Reload your terminal just in case, I assume that catkin_ws is sourced in bash.src.

To execute and complete part 'a' of task 1 first:

In one tab load up the gazebo competition using:

rosrun osrf_gear gear.py -f 'catkin_find --share osrf_gear'/config/qual1a.yaml '~/catkin_ws/src/ariac_qual_1/config/qual_1_conf.yaml' 

In another tab load up MoveIt! using:

roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true

and finally when it is all up and running, just execute the script in another tab:

rosrun ariac_qual_1 /script/ariac_qual_1.py

It should now get an order for 5 parts, (2 wrench, 3 cogs). The arm will move to pick up a part (a bit flaky it can take a while) and then move to the left tray and place it down.

Once complete, and all 5 parts are in the tray, the arm returns to the middle, the AGV return command is sent to finish and in the first tab you should see something like this pop up.

End of trial. Final score: 10
Score breakdown:
<game_score>
Total score: [10]
Total process time: [118.448]
Part travel time: [56.024]
<order_score order_0>
Total score: [10]
Time taken: [0]
Complete: [true]
<tray_score >
Total score: [10]
Complete: [false]
Submitted: [true]
Part presence score: [5]
All parts bonus: [5]
Part pose score: [0]
</tray_score>
</order_score>
</game_score>

At this point, close everything down and follow the instructions on keeping the logs to later upload, details here:
https://bitbucket.org/osrf/ariac/wiki/qualifiers/qual1

One small caveat, I'm pretty sure that you need to go to the .gazebo/server-11345/default.log and use this instead of the performance.log found in .ariac/log/performance.log, is this is only symbolically linked / shortcut so will get overwritten.

You can review the gazebo log using the following command:

roslaunch osrf_gear gear_playback.launch

If all went well then you can move on to part 'b' of the task.

FIRST! open up the ariac_qual_1.py script and go to line 40-41 and change

 agv = 'agv1' to  agv = 'agv2'
 tray = 'Left' to tray = 'Right'

NEXT rerun as before however use this command in the first tab to run the part 'b' task instead.

rosrun osrf_gear gear.py -f 'catkin_find --share osrf_gear'/config/qual1b.yaml '~/catkin_ws/src/ariac_qual_1/config/qual_1_conf.yaml'

In this task the first part will fall off just before it is placed in the right tray, this is part of the challenge so don't worry, the script knows to go back and start again.

All the best with the competition, it will only get harder from here on out.

ERRORS:

Sometimes at loadup of gazebo the arm may not be in its default state, (it rests on the crate), this can cause it to knock around some of the parts in the beginning and because I don't have a dynamic update, it will never find them, so you will have to shut down and restart.

Also some times when running the script it can fail to find the 'world' frame. I'm not sure why, I should have probably added something to check. At any rate, just keep re-running the script until it works :)
