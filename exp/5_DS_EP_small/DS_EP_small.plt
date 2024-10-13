set out "DS_EP_small.eps"
set terminal postscript portrait color enhanced "Helvetica" 25

set size 2.5, 1.07
set pointsize 3

set multiplot
#===========================================================
#Note the NaN variable

set size 2.5, 0.1;
set origin 0, 1.01;

set key center top horizontal samplen 2
set yrange [0.0000000001:0.0000000002]

unset border
unset tics
#unset label

plot NaN title 'SE-Oracle-Adapt' with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
NaN title 'EAR-Oracle-Adapt' with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
NaN title 'RC-Oracle-Naive' with linespoints linetype 1 pointtype 6 linecolor rgbcolor "#000000", \
NaN title 'RC-Oracle' with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000", \
NaN title 'CH-Adapt' with linespoints linetype 1 pointtype 8 linecolor rgbcolor "#808080", \
NaN title 'Kaul-Adapt' with linespoints linetype 1 pointtype 10 linecolor rgbcolor "#808080", \
NaN title 'Dijk-Adapt' with linespoints linetype 1 pointtype 12 linecolor rgbcolor "#808080", \
NaN title 'FastFly' with linespoints linetype 1 pointtype 14 linecolor rgbcolor "#0000FF"

set border
set tics


#=========================================
set size 0.7, 0.5;
set origin 0, 0.47;

set xlabel "Dataset size (k)" offset 0,0.1,0
set ylabel "Construction Time (s)" offset 0.9,0,0
set key above
set log y

set xrange [9: 51]
set yrange [0.01: 350000] 
set label 11 center at graph 0.5,char 1 "(a)" offset 0,13.9,0
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("10" 10, "20" 20, "30" 30, "40" 40, "50" 50)

plot "1_SE_Oracle_Adapt.txt" using (floor($2/10000)*10):(($5+$6)/1000) notitle with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"2_EAR_Oracle_Adapt.txt" using (floor($2/10000)*10):(($5+$6)/1000) notitle with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"3_RC_Oracle_Naive.txt" using (floor($2/10000)*10):($6/1000) notitle with linespoints linetype 1 pointtype 6 linecolor rgbcolor "#000000", \
"4_RC_Oracle.txt" using (floor($2/10000)*10):($6/1000) notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 0.6, 0.47;

set xlabel "Dataset size (k)"
set ylabel "Memory (MB)"
set key above
set log y

set xrange [9: 51]
set yrange [0.01: 20000]
set label 11 center at graph 0.5,char 1 "(b)" 
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("10" 10, "20" 20, "30" 30, "40" 40, "50" 50)

plot "1_SE_Oracle_Adapt.txt" using (floor($2/10000)*10):($10+$11) notitle with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"2_EAR_Oracle_Adapt.txt" using (floor($2/10000)*10):($10+$11) notitle with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"3_RC_Oracle_Naive.txt" using (floor($2/10000)*10):11 notitle with linespoints linetype 1 pointtype 6 linecolor rgbcolor "#000000", \
"5_CH_Adapt.txt" using (floor($2/10000)*10):($10+$11) notitle with linespoints linetype 1 pointtype 8 linecolor rgbcolor "#808080", \
"6_Kaul_Adapt.txt" using (floor($2/10000)*10):($10+$11) notitle with linespoints linetype 1 pointtype 10 linecolor rgbcolor "#808080", \
"7_Dijk_Adapt.txt" using (floor($2/10000)*10):($10+$11) notitle with linespoints linetype 1 pointtype 12 linecolor rgbcolor "#808080", \
"8_FastFly.txt" using (floor($2/10000)*10):11 notitle with linespoints linetype 1 pointtype 14 linecolor rgbcolor "#0000FF", \
"4_RC_Oracle.txt" using (floor($2/10000)*10):11 notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 1.2, 0.47;

set xlabel "Dataset size (k)"
set ylabel "Size (MB)"
set key above
set log y

set xrange [9: 51]
set yrange [0.05: 1000]
set label 11 center at graph 0.5,char 1 "(c)" 
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("10" 10, "20" 20, "30" 30, "40" 40, "50" 50)

plot "1_SE_Oracle_Adapt.txt" using (floor($2/10000)*10):12 notitle with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"2_EAR_Oracle_Adapt.txt" using (floor($2/10000)*10):12 notitle with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"3_RC_Oracle_Naive.txt" using (floor($2/10000)*10):12 notitle with linespoints linetype 1 pointtype 6 linecolor rgbcolor "#000000", \
"4_RC_Oracle.txt" using (floor($2/10000)*10):12 notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 1.8, 0.47;

set xlabel "Dataset size (k)"
set ylabel "Query Time (ms)"
set key above
set log y

set xrange [9: 51]
set yrange [0.0005: 100000]
set label 11 center at graph 0.5,char 1 "(d)" 
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("10" 10, "20" 20, "30" 30, "40" 40, "50" 50)

plot "1_SE_Oracle_Adapt.txt" using (floor($2/10000)*10):7:9:8 notitle with yerrorlines linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"2_EAR_Oracle_Adapt.txt" using (floor($2/10000)*10):7:9:8 notitle with yerrorlines linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"3_RC_Oracle_Naive.txt" using (floor($2/10000)*10):7:9:8 notitle with yerrorlines linetype 1 pointtype 6 linecolor rgbcolor "#000000", \
"5_CH_Adapt.txt" using (floor($2/10000)*10):($5+$7):($5+$9):($5+$8) notitle with yerrorlines linetype 1 pointtype 8 linecolor rgbcolor "#808080", \
"6_Kaul_Adapt.txt" using (floor($2/10000)*10):($5+$7):($5+$9):($5+$8) notitle with yerrorlines linetype 1 pointtype 10 linecolor rgbcolor "#808080", \
"7_Dijk_Adapt.txt" using (floor($2/10000)*10):($5+$7):($5+$9):($5+$8) notitle with yerrorlines linetype 1 pointtype 12 linecolor rgbcolor "#808080", \
"8_FastFly.txt" using (floor($2/10000)*10):7:9:8 notitle with yerrorlines linetype 1 pointtype 14 linecolor rgbcolor "#0000FF", \
"4_RC_Oracle.txt" using (floor($2/10000)*10):7:9:8 notitle with yerrorlines linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 0.3, -0.02;

set xlabel "Dataset size (k)"
set ylabel "kNN Query Time (ms)"
set key above
set log y

set xrange [9: 51]
set yrange [0.3: 5000000]
set label 11 center at graph 0.5,char 1 "(e)" offset 0,-0.4,0
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("10" 10, "20" 20, "30" 30, "40" 40, "50" 50)

plot "1_SE_Oracle_Adapt.txt" using (floor($2/10000)*10):($3*$3*$7) notitle with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"2_EAR_Oracle_Adapt.txt" using (floor($2/10000)*10):($3*$3*$7) notitle with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"3_RC_Oracle_Naive.txt" using (floor($2/10000)*10):($3*$3*$7) notitle with linespoints linetype 1 pointtype 6 linecolor rgbcolor "#000000", \
"5_CH_Adapt.txt" using (floor($2/10000)*10):($3*($5+$7)) notitle with linespoints linetype 1 pointtype 8 linecolor rgbcolor "#808080", \
"6_Kaul_Adapt.txt" using (floor($2/10000)*10):($3*($5+$7)) notitle with linespoints linetype 1 pointtype 10 linecolor rgbcolor "#808080", \
"7_Dijk_Adapt.txt" using (floor($2/10000)*10):($3*($5+$7)) notitle with linespoints linetype 1 pointtype 12 linecolor rgbcolor "#808080", \
"8_FastFly.txt" using (floor($2/10000)*10):($3*$7) notitle with linespoints linetype 1 pointtype 14 linecolor rgbcolor "#0000FF", \
"4_RC_Oracle.txt" using (floor($2/10000)*10):($3*$3*$7) notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 0.9, -0.02;

set xlabel "Dataset size (k)"
set ylabel "Range Query Time (ms)"
set key above
set log y

set xrange [9: 51]
set yrange [0.3: 5000000]
set label 11 center at graph 0.5,char 1 "(e)" offset 0,-0.4,0
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("10" 10, "20" 20, "30" 30, "40" 40, "50" 50)

plot "1_SE_Oracle_Adapt.txt" using (floor($2/10000)*10):($3*$3*$7) notitle with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"2_EAR_Oracle_Adapt.txt" using (floor($2/10000)*10):($3*$3*$7) notitle with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"3_RC_Oracle_Naive.txt" using (floor($2/10000)*10):($3*$3*$7) notitle with linespoints linetype 1 pointtype 6 linecolor rgbcolor "#000000", \
"5_CH_Adapt.txt" using (floor($2/10000)*10):($3*($5+$7)) notitle with linespoints linetype 1 pointtype 8 linecolor rgbcolor "#808080", \
"6_Kaul_Adapt.txt" using (floor($2/10000)*10):($3*($5+$7)) notitle with linespoints linetype 1 pointtype 10 linecolor rgbcolor "#808080", \
"7_Dijk_Adapt.txt" using (floor($2/10000)*10):($3*($5+$7)) notitle with linespoints linetype 1 pointtype 12 linecolor rgbcolor "#808080", \
"8_FastFly.txt" using (floor($2/10000)*10):($3*$7) notitle with linespoints linetype 1 pointtype 14 linecolor rgbcolor "#0000FF", \
"4_RC_Oracle.txt" using (floor($2/10000)*10):($3*$3*$7) notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 1.5, -0.02;

set xlabel "Dataset size (k)"
set ylabel "Distance Error"
set key above
unset log y

set xrange [9: 51]
set yrange [0: 0.12]
set label 11 center at graph 0.5,char 1 "(g)" 
set bmargin 5
set format x "%g"
set format y "%g"

set xtics ("10" 10, "20" 20, "30" 30, "40" 40, "50" 50)

plot "1_SE_Oracle_Adapt.txt" using (floor($2/10000)*10):14 notitle with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"2_EAR_Oracle_Adapt.txt" using (floor($2/10000)*10):14 notitle with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"3_RC_Oracle_Naive.txt" using (floor($2/10000)*10):14 notitle with linespoints linetype 1 pointtype 6 linecolor rgbcolor "#000000", \
"5_CH_Adapt.txt" using (floor($2/10000)*10):14 notitle with linespoints linetype 1 pointtype 8 linecolor rgbcolor "#808080", \
"6_Kaul_Adapt.txt" using (floor($2/10000)*10):14 notitle with linespoints linetype 1 pointtype 10 linecolor rgbcolor "#808080", \
"7_Dijk_Adapt.txt" using (floor($2/10000)*10):14 notitle with linespoints linetype 1 pointtype 12 linecolor rgbcolor "#808080", \
"8_FastFly.txt" using (floor($2/10000)*10):14 notitle with linespoints linetype 1 pointtype 14 linecolor rgbcolor "#0000FF", \
"4_RC_Oracle.txt" using (floor($2/10000)*10):14 notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


unset multiplot
