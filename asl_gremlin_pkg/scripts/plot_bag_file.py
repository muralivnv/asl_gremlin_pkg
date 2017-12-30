#! /usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
import sys, os

## robot_name
file_path  = os.path.realpath(__file__)
wrk_spc    = file_path[:file_path.find("src/") + len("src/")]

robot_name_file = open( wrk_spc + "bash_scripts/this_robot_name.sh", "r")
line_list = robot_name_file.readlines()
robot_name_file.close()
robot_name = line_list[-1].split('=')
robot_name = ("/" + robot_name[1]).replace('"','')

if robot_name == "" or robot_name == "/":
	robot_name = "/asl_gremlin1"

## Read bagfile
bag = rosbag.Bag(sys.argv[1])

## extract reference trajectory
ref_traj = [[msg.x, msg.x_dot, msg.x_ddot, msg.y, msg.y_dot ,msg.y_ddot, msg.theta,t] for (topic,msg,t) in
                        bag.read_messages(topics=[robot_name+"/trajectory_generation/reference_trajectory"])]

x_ref       = [i[0] for i in ref_traj]
x_dot_ref   = [i[1] for i in ref_traj]
x_ddot_ref  = [i[2] for i in ref_traj]

y_ref       = [i[3] for i in ref_traj]
y_dot_ref   = [i[4] for i in ref_traj]
y_ddot_ref  = [i[5] for i in ref_traj]

theta_ref   = [i[6] for i in ref_traj]
t_ref       = [i[7] for i in ref_traj] 
t_ref       = [(t-t_ref[0]).to_sec() for t in t_ref]

## extract actual rover pose
actual_state = [[msg.pose.point.x, msg.pose.point.y, msg.heading,t] for (topic, msg, t) in 

                        bag.read_messages(topics=[robot_name+"/state_feedback/selected_feedback"])]
x_act       = [i[0] for i in actual_state]
y_act       = [i[1] for i in actual_state]
theta_act   = [i[2] for i in actual_state]
t_act       = [i[3] for i in actual_state]
t_act       = [(t-t_act[0]).to_sec() for t in t_act]

### extract commanded angular velocities

cmd_ang_vel = [[msg.wl, msg.wr,t] for (topic, msg, t) in
                    bag.read_messages(topics=[robot_name+"/controller/cmd_angular_vel"])]

wl_cmd      = [i[0] for i in cmd_ang_vel]
wr_cmd      = [i[1] for i in cmd_ang_vel]
t_omega_cmd     = [i[2] for i in cmd_ang_vel]
t_omega_cmd     = [(t-t_omega_cmd[0]).to_sec() for t in t_omega_cmd]

### extract actual angular velocities

actual_ang_vel = [[msg.wl, msg.wr,t] for (topic, msg, t) in
                    bag.read_messages(topics=[robot_name+"/state_feedback/encoder/actual_ang_vel"])]

wl_act      = [i[0] for i in actual_ang_vel]
wr_act      = [i[1] for i in actual_ang_vel]
t_omega     = [i[2] for i in actual_ang_vel]
t_omega     = [(t-t_omega[0]).to_sec() for t in t_omega]

### Close rosbag
bag.close()

## Plotting
# plot (X-ref, Y-ref, X-actual, Y-actual)
plt.figure(1)
plt.hold(True)
plt.grid(True)
ref_path,=plt.plot(x_ref,y_ref,"r",linewidth=1.5, label="Reference Path")
act_path,=plt.plot(x_act,y_act,"b",linewidth=1,label="Actual Path")
plt.legend(handles=[ref_path,act_path])
plt.title("Desired and Actual path of the Rover",fontweight='bold')
plt.xlabel("x (m)",fontweight='bold',fontsize=12)
plt.ylabel("y (m)")

# plot (t, X-ref, t, X-actual)
plt.figure(2)
plt.hold(True)
ref_path,=plt.plot(t_ref,x_ref,"b",linewidth=1.5, label="X-Ref")
act_path,=plt.plot(t_act,x_act,"r",linewidth=1.5, label="X-Act")
plt.legend(handles=[ref_path, act_path])
plt.grid(True)
plt.xlabel("Time, sec",fontweight='bold', fontsize=12)
plt.ylabel("X (m)",fontweight='bold', fontsize=12)
plt.title("Trajectory tracking in X-Direction",fontweight='bold');

# plot (t, Y-ref, t, Y-actual)
plt.figure(3)
plt.hold(True)
ref_path,=plt.plot(t_ref,y_ref,"b",linewidth=1.5, label="Y-Ref")
act_path,=plt.plot(t_act,y_act,"r",linewidth=1.5, label="Y-Act")
plt.legend(handles=[ref_path, act_path])
plt.grid(True)
plt.xlabel("Time, sec",fontweight='bold', fontsize=12)
plt.ylabel("Y (m)",fontweight='bold', fontsize=12)
plt.title("Trajectory tracking in Y-Direction",fontweight='bold');

# angular vel
plt.figure(4)
plt.hold(True)
ref_ang_vel,=plt.plot(t_omega_cmd,wl_cmd,"b",linewidth=1.5, label="Wl-cmd")
act_ang_vel,=plt.plot(t_omega,wl_act,"r",linewidth=1.5, label="Wl-act")
plt.legend(handles=[ref_ang_vel, act_ang_vel])
plt.grid(True)
plt.xlabel("Time, sec",fontweight='bold', fontsize=12)
plt.ylabel("Wl (rad/sec)",fontweight='bold', fontsize=12)
plt.title("Commanded and actual angular velocity of left-wheel",fontweight='bold');


# angular vel
plt.figure(5)
plt.hold(True)
ref_ang_vel,=plt.plot(t_omega_cmd,wr_cmd,"b",linewidth=1.5, label="Wr-cmd")
act_ang_vel,=plt.plot(t_omega,wr_act,"r",linewidth=1.5, label="Wr-act")
plt.legend(handles=[ref_ang_vel, act_ang_vel])
plt.grid(True)
plt.xlabel("Time, sec",fontweight='bold', fontsize=12)
plt.ylabel("Wr (rad/sec)",fontweight='bold', fontsize=12)
plt.title("Commanded and actual angular velocity of right-wheel",fontweight='bold');

####################
plt.show()

