set term qt 0
plot "../../matlab_data/omega_from_encoder.txt" u 2 w l t "from MATLAB", "../converted_encoder_to_omega.dat" u 1 w l t "from C++"
set title "Left wheel ang vel from encoder (rad/sec)"
set grid
set xlabel "Measurement Index"
set ylabel "$omega _l"
replot

set term qt 1
plot "../../matlab_data/omega_from_encoder.txt" u 3 w l t "from MATLAB", "../converted_encoder_to_omega.dat" u 2 w l t "from C++"
set title "Right wheel ang vel from encoder (rad/sec)"
set grid
set xlabel "Measurement Index"
set ylabel "omega _r"
replot
