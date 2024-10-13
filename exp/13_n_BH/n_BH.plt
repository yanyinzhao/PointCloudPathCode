set out "n_BH.eps"
set terminal postscript portrait color enhanced "Helvetica" 25

set size 2.5, 0.99
set pointsize 3

set multiplot
#===========================================================
#Note the NaN variable

set size 2.5, 0.1;
set origin 0, 0.93;

set key center top horizontal samplen 2
set yrange [0.0000000001:0.0000000002]

unset border
unset tics
#unset label

plot NaN title 'RC-Oracle' with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000", \
NaN title 'CH-Adapt' with linespoints linetype 1 pointtype 8 linecolor rgbcolor "#808080", \
NaN title 'Kaul-Adapt' with linespoints linetype 1 pointtype 10 linecolor rgbcolor "#808080", \
NaN title 'Dijk-Adapt' with linespoints linetype 1 pointtype 12 linecolor rgbcolor "#808080", \
NaN title 'FastFly' with linespoints linetype 1 pointtype 14 linecolor rgbcolor "#0000FF"

set border
set tics


#=========================================
set size 0.7, 0.5;
set origin 0, 0.47;

set xlabel "POI number (k)" offset 0,0.1,0
set ylabel "Construction Time (s)" offset 0.9,0,0
set key above
#set log y

set xrange [450: 2550]
set yrange [0: 120] 
set label 11 center at graph 0.5,char 1 "(a)" offset 0,13.9,0
set bmargin 5
set format x "%g"
set format y "%g"

set xtics ("0.5" 500, "1" 1000, "1.5" 1500, "2" 2000, "2.5" 2500)

plot "4_RC_Oracle.txt" using 3:($6/1000) notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 0.6, 0.47;

set xlabel "POI number (k)"
set ylabel "Memory (MB)"
set key above
set log y

set xrange [450: 2550]
set yrange [3: 10000]
set label 11 center at graph 0.5,char 1 "(b)" 
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("0.5" 500, "1" 1000, "1.5" 1500, "2" 2000, "2.5" 2500)

plot "5_CH_Adapt.txt" using 3:($10+$11) notitle with linespoints linetype 1 pointtype 8 linecolor rgbcolor "#808080", \
"6_Kaul_Adapt.txt" using 3:($10+$11) notitle with linespoints linetype 1 pointtype 10 linecolor rgbcolor "#808080", \
"7_Dijk_Adapt.txt" using 3:($10+$11) notitle with linespoints linetype 1 pointtype 12 linecolor rgbcolor "#808080", \
"8_FastFly.txt" using 3:11 notitle with linespoints linetype 1 pointtype 14 linecolor rgbcolor "#0000FF", \
"4_RC_Oracle.txt" using 3:11 notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 1.2, 0.47;

set xlabel "POI number (k)"
set ylabel "Size (MB)"
set key above
set log y

set xrange [450: 2550]
set yrange [30: 4000]
set label 11 center at graph 0.5,char 1 "(c)" 
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("0.5" 500, "1" 1000, "1.5" 1500, "2" 2000, "2.5" 2500)

plot "4_RC_Oracle.txt" using 3:12 notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 1.8, 0.47;

set xlabel "POI number (k)"
set ylabel "Query Time (ms)"
set key above
set log y

set xrange [450: 2550]
set yrange [0.001: 1000000]
set label 11 center at graph 0.5,char 1 "(d)" 
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("0.5" 500, "1" 1000, "1.5" 1500, "2" 2000, "2.5" 2500)

plot "5_CH_Adapt.txt" using 3:($5+$7):($5+$9):($5+$8) notitle with yerrorlines linetype 1 pointtype 8 linecolor rgbcolor "#808080", \
"6_Kaul_Adapt.txt" using 3:($5+$7):($5+$9):($5+$8) notitle with yerrorlines linetype 1 pointtype 10 linecolor rgbcolor "#808080", \
"7_Dijk_Adapt.txt" using 3:($5+$7):($5+$9):($5+$8) notitle with yerrorlines linetype 1 pointtype 12 linecolor rgbcolor "#808080", \
"8_FastFly.txt" using 3:7:9:8 notitle with yerrorlines linetype 1 pointtype 14 linecolor rgbcolor "#0000FF", \
"4_RC_Oracle.txt" using 3:7:9:8 notitle with yerrorlines linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 0.3, -0.02;

set xlabel "POI number (k)"
set ylabel "kNN Query Time (ms)"
set key above
set log y

set xrange [450: 2550]
set yrange [300: 1000000000]
set label 11 center at graph 0.5,char 1 "(e)" offset 0,-0.4,0
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("0.5" 500, "1" 1000, "1.5" 1500, "2" 2000, "2.5" 2500)

plot "5_CH_Adapt.txt" using 3:($3*($5+$7)) notitle with linespoints linetype 1 pointtype 8 linecolor rgbcolor "#808080", \
"6_Kaul_Adapt.txt" using 3:($3*($5+$7)) notitle with linespoints linetype 1 pointtype 10 linecolor rgbcolor "#808080", \
"7_Dijk_Adapt.txt" using 3:($3*($5+$7)) notitle with linespoints linetype 1 pointtype 12 linecolor rgbcolor "#808080", \
"8_FastFly.txt" using 3:($3*$7) notitle with linespoints linetype 1 pointtype 14 linecolor rgbcolor "#0000FF", \
"4_RC_Oracle.txt" using 3:($3*$3*$7) notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 0.9, -0.02;

set xlabel "POI number (k)"
set ylabel "Range Query Time (ms)"
set key above
set log y

set xrange [450: 2550]
set yrange [300: 1000000000]
set label 11 center at graph 0.5,char 1 "(f)" 
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("0.5" 500, "1" 1000, "1.5" 1500, "2" 2000, "2.5" 2500)

plot "5_CH_Adapt.txt" using 3:($3*($5+$7)) notitle with linespoints linetype 1 pointtype 8 linecolor rgbcolor "#808080", \
"6_Kaul_Adapt.txt" using 3:($3*($5+$7)) notitle with linespoints linetype 1 pointtype 10 linecolor rgbcolor "#808080", \
"7_Dijk_Adapt.txt" using 3:($3*($5+$7)) notitle with linespoints linetype 1 pointtype 12 linecolor rgbcolor "#808080", \
"8_FastFly.txt" using 3:($3*$7) notitle with linespoints linetype 1 pointtype 14 linecolor rgbcolor "#0000FF", \
"4_RC_Oracle.txt" using 3:($3*$3*$7) notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 1.5, -0.02;

set xlabel "POI number (k)"
set ylabel "Distance Error"
set key above
unset log y

set xrange [450: 2550]
set yrange [0: 0.12]
set label 11 center at graph 0.5,char 1 "(g)" 
set bmargin 5
set format x "%g"
set format y "%g"

set xtics ("0.5" 500, "1" 1000, "1.5" 1500, "2" 2000, "2.5" 2500)

plot "5_CH_Adapt.txt" using 3:14 notitle with linespoints linetype 1 pointtype 8 linecolor rgbcolor "#808080", \
"6_Kaul_Adapt.txt" using 3:14 notitle with linespoints linetype 1 pointtype 10 linecolor rgbcolor "#808080", \
"7_Dijk_Adapt.txt" using 3:14 notitle with linespoints linetype 1 pointtype 12 linecolor rgbcolor "#808080", \
"8_FastFly.txt" using 3:14 notitle with linespoints linetype 1 pointtype 14 linecolor rgbcolor "#0000FF", \
"4_RC_Oracle.txt" using 3:14 notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


unset multiplot
