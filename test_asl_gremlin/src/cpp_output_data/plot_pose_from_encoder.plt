set term qt 0
plot "pose_from_encoder.txt" u 1:2 w l t "from C++", "../matlab_data/pose_encoder.txt" u 2:3 w l t "from MATLAB"
set grid
set title "Pose information from encoder ticks integration"
set xlabel "X(m)"
set ylabel "Y(m)"
replot
