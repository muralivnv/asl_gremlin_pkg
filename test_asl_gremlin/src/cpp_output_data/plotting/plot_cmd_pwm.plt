set term qt 0
plot "../cmd_pwm.dat" u 1 w l t "from C++", "../../matlab_data/cmd_pwm_matlab.txt" u 2 w l t "from MATLAB"
set grid
set title "Commanded Motor PWM(left)"
set xlabel "Measurement Index"
set ylabel "PWM_l"
replot

set term qt 1
plot "../cmd_pwm.dat" u 2 w l t "from C++", "../../matlab_data/cmd_pwm_matlab.txt" u 3 w l t "from MATLAB"
set grid
set title "Commanded Motor PWM(right)"
set xlabel "Measurement Index"
set ylabel "PWM_r"
replot
