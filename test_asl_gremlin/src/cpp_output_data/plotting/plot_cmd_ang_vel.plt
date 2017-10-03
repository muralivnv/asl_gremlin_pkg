set term qt 0
plot "../cmd_ang_vel.dat" u 1 w l t "from C++", "../../matlab_data/cmd_ang_vel_matlab.txt" u 2 w l t "from MATLAB"
set title "Commanded Angular Velocity(left, rad/sec)"
set xlabel "Measurement Index"
set ylabel "Omega_l"
set grid
replot

set term qt 1
plot "../cmd_ang_vel.dat" u 2 w l t "from C++", "../../matlab_data/cmd_ang_vel_matlab.txt" u 3 w l t "from MATLAB"
set title "Commanded Angular Velocity(right, rad/sec)"
set xlabel "Measurement Index"
set ylabel "Omega_r"
set grid
replot
