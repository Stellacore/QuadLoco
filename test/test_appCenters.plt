#
# Plot file generated by test_appCenters.cpp
#

# Run with e.g.: 
# $>  gnuplot -geometry 1000x1000  ~/repos/QuadLoco/test/test_appCenters.plt

#set palette maxcolors 5
set palette defined \
	(0 "green", 1 "yellow", 2 "cyan", 3 "orange", 5 "blue")

set xlabel "X location [m]"
set ylabel "Y location [m]"
set zlabel "Z location [m]" rotate parallel
set cblabel "Center Diff [pix]"

set xyplane 0
set cbrange [0:5]

splot [][][] \
	  'test_appCenters.dat' u 2:3:4 pt 7 lc 0 ps .5  \
	, 'test_appCenters.dat' u 2:3:4:9 palette pt 7 ps 2. \
	;
pause -1;


