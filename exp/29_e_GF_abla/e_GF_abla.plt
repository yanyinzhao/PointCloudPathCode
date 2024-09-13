set out "e_GF_abla.eps"
set terminal postscript portrait color enhanced "Helvetica" 25

set size 2.5, 1.04
set pointsize 3

set multiplot
#===========================================================

set size 2.5, 0.1;
set origin 0, 0.98;

set key center top horizontal samplen 2
set yrange [0.0000000001:0.0000000002]

unset border
unset tics

plot NaN title 'SE-Oracle-FastFly-Adapt' with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
NaN title 'EAR-Oracle-FastFly-Adapt' with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
NaN title 'RC-Oracle' with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000", \
NaN title 'Theoretical bound ({/Symbol e})' with lines dashtype 2 linecolor rgbcolor "black"

set border
set tics

#=========================================
set size 0.7, 0.5;
set origin 0, 0.47;

set xlabel "{/Symbol e}" offset 0,0.1,0
set ylabel "Construction Time (s)" offset 0.9,0,0
set key above
set log y

set xrange [-0.02: 1.02]
set yrange [5: 10000] 
set xtics rotate by 45 right
set label 11 center at graph 0.5,char 1 "(a)" offset 0,13.9,0
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("0" 0, "0.2" 0.2, "0.4" 0.4, "0.6" 0.6, "0.8" 0.8, "1" 1)

plot "9_SE_Oracle_FastFly_Adapt.txt" using 4:(($5+$6)/1000) notitle with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"10_EAR_Oracle_FastFly_Adapt.txt" using 4:(($5+$6)/1000) notitle with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"4_RC_Oracle.txt" using 4:($6/1000) notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 0.6, 0.47;

set xlabel "{/Symbol e}"
set ylabel "Memory (MB)"
set key above
set log y

set xrange [-0.02: 1.02]
set yrange [10: 50000]
set label 11 center at graph 0.5,char 1 "(b)" 
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("0" 0, "0.2" 0.2, "0.4" 0.4, "0.6" 0.6, "0.8" 0.8, "1" 1)

plot "9_SE_Oracle_FastFly_Adapt.txt" using 4:($8+$9) notitle with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"10_EAR_Oracle_FastFly_Adapt.txt" using 4:($8+$9) notitle with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"4_RC_Oracle.txt" using 4:($8+$9) notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 1.2, 0.47;

set xlabel "{/Symbol e}"
set ylabel "Size (MB)"
set key above
set log y

set xrange [-0.02: 1.02]
set yrange [10: 50000]
set label 11 center at graph 0.5,char 1 "(c)" 
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("0" 0, "0.2" 0.2, "0.4" 0.4, "0.6" 0.6, "0.8" 0.8, "1" 1)

plot "9_SE_Oracle_FastFly_Adapt.txt" using 4:10 notitle with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"10_EAR_Oracle_FastFly_Adapt.txt" using 4:10 notitle with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"4_RC_Oracle.txt" using 4:10 notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 1.8, 0.47;

set xlabel "{/Symbol e}"
set ylabel "Query Time (ms)"
set key above
set log y

set xrange [-0.02: 1.02]
set yrange [0.003: 1]
set label 11 center at graph 0.5,char 1 "(d)" 
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("0" 0, "0.2" 0.2, "0.4" 0.4, "0.6" 0.6, "0.8" 0.8, "1" 1)

plot "9_SE_Oracle_FastFly_Adapt.txt" using 4:7 notitle with yerrorlines linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"10_EAR_Oracle_FastFly_Adapt.txt" using 4:7 notitle with yerrorlines linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"4_RC_Oracle.txt" using 4:7 notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 0.3, -0.02;

set xlabel "{/Symbol e}"
set ylabel "kNN Query Time (ms)"
set key above
set log y

set xrange [-0.02: 1.02]
set yrange [500: 500000]
set label 11 center at graph 0.5,char 1 "(e)" offset 0,-0.4,0
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("0" 0, "0.2" 0.2, "0.4" 0.4, "0.6" 0.6, "0.8" 0.8, "1" 1)

plot "9_SE_Oracle_FastFly_Adapt.txt" using 4:($3*$3*$7) notitle with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"10_EAR_Oracle_FastFly_Adapt.txt" using 4:($3*$3*$7) notitle with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"4_RC_Oracle.txt" using 4:($3*$3*$7) notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 0.9, -0.02;

set xlabel "{/Symbol e}"
set ylabel "Range Query Time (ms)"
set key above
set log y

set xrange [-0.02: 1.02]
set yrange [500: 500000]
set label 11 center at graph 0.5,char 1 "(f)" 
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("0" 0, "0.2" 0.2, "0.4" 0.4, "0.6" 0.6, "0.8" 0.8, "1" 1)

plot "9_SE_Oracle_FastFly_Adapt.txt" using 4:($3*$3*$7) notitle with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"10_EAR_Oracle_FastFly_Adapt.txt" using 4:($3*$3*$7) notitle with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"4_RC_Oracle.txt" using 4:($3*$3*$7) notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 1.5, -0.02;

set xlabel "{/Symbol e}"
set ylabel "Distance Error"
set key above
unset log y

set xrange [-0.02: 1.02]
set yrange [-0.02: 1.02]
set label 11 center at graph 0.5,char 1 "(g)" 
set bmargin 5
set format x "%g"
set format y "%g"

set xtics ("0" 0, "0.2" 0.2, "0.4" 0.4, "0.6" 0.6, "0.8" 0.8, "1" 1)
set ytics ("0" 0, "0.2" 0.2, "0.4" 0.4, "0.6" 0.6, "0.8" 0.8, "1" 1)

plot "9_SE_Oracle_FastFly_Adapt.txt" using 4:12 notitle with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"10_EAR_Oracle_FastFly_Adapt.txt" using 4:12 notitle with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"4_RC_Oracle.txt" using 4:12 notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000", \
x notitle with lines dashtype 2 linecolor rgbcolor "black"


unset multiplot
