#! /usr/bin/env python

import rosbag
import matplotlib.pyplot as plt

bag = rosbag.Bag('/home/vnv/test.bag')
ref_traj = [[msg.x, msg.x_dot, msg.x_ddot, msg.y, msg.y_dot ,msg.y_ddot] for (topic,msg,t) in bag.read_messages(topics=["/asl_gremlin1/trajectory_generation/reference_trajectory"])]

x       = [i[0] for i in ref_traj]
x_dot   = [i[1] for i in ref_traj]
x_ddot  = [i[2] for i in ref_traj]

y       = [i[3] for i in ref_traj]
y_dot   = [i[4] for i in ref_traj]
y_ddot  = [i[5] for i in ref_traj]


plt.figure

plt.plot(x,y,"b")
plt.grid()
plt.xlabel("X(m)")
plt.ylabel("Y(m)")
plt.show()

bag.close()
