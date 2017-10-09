#! /usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
import sys

## Read bagfile
bag = rosbag.Bag('/home/vnv/'+sys.argv[1])

## extract reference trajectory
ref_traj = [[msg.x, msg.x_dot, msg.x_ddot, msg.y, msg.y_dot ,msg.y_ddot, msg.theta,t] for (topic,msg,t) in
                        bag.read_messages(topics=["/asl_gremlin1/trajectory_generation/reference_trajectory"])]

x_ref       = [i[0] for i in ref_traj]
x_dot_ref   = [i[1] for i in ref_traj]
x_ddot_ref  = [i[2] for i in ref_traj]

y_ref       = [i[3] for i in ref_traj]
y_dot_ref   = [i[4] for i in ref_traj]
y_ddot_ref  = [i[5] for i in ref_traj]

theta_ref   = [i[6] for i in ref_traj]
t_ref       = [i[7] for i in ref_traj] 
t_ref       = [ (t-t_ref[0]).to_sec() for t in t_ref]

## extract actual rover pose
#actual_state = [[msg.pose.point.x, msg.pose.point.y, msg.heading,t] for (topic, msg, t) in 
#                        bag.read_messages(topics=["/asl_gremlin1/state_feedback/selected_feedback"])

#x_act       = [i[0] for i in actual_state]
#y_act       = [i[1] for i in actual_state]
#theta_act   = [i[2] for i in actual_state]
#t_act       = [i[3] for i in actual_state]
#
### extract actual angular velocities
#
#actual_ang_vel = [[msg.wl, msg.wr] for (topic, msg, t) in
#                    bag.read_messages(topics=["/asl_gremlin1/state_feedback/encoder/actual_ang_vel"])
#
#wl_act      = [i[0] for i in actual_ang_vel]
#wr_act      = [i[1] for i in actual_ang_vel]
#t_omega     = [i[2] for i in actual_ang_vel]

### Close rosbag
bag.close()

## Plotting
# plot (X-ref, Y-ref, X-actual, Y-actual)
plt.figure(1)
plt.plot(x_ref,y_ref,"r-.",linewidth=1.5)
plt.grid()
plt.xlabel("x(m)")
plt.ylabel("y(m)")

# plot (t, X-ref, t, X-actual)
plt.figure(2)
plt.plot(t_ref,x_ref,"b")
plt.grid()
plt.xlabel("Time(sec)")
plt.ylabel("x(m)")

# plot (t, Y-ref, t, Y-actual)
plt.figure(3)
plt.plot(t_ref,y_ref,"b")
plt.grid()
plt.xlabel("Time(sec)")
plt.ylabel("y(m)")

####################
plt.show()

