set out "n_EP_small_TIN.eps"
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

plot NaN title 'SE-Oracle' with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
NaN title 'EAR-Oracle' with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
NaN title 'RC-Oracle-Naive-Adapt' with linespoints linetype 1 pointtype 6 linecolor rgbcolor "#000000", \
NaN title 'RC-Oracle-Adapt' with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000", \
NaN title 'CH' with linespoints linetype 1 pointtype 8 linecolor rgbcolor "#808080", \
NaN title 'Kaul' with linespoints linetype 1 pointtype 10 linecolor rgbcolor "#808080", \
NaN title 'Dijk' with linespoints linetype 1 pointtype 12 linecolor rgbcolor "#808080", \
NaN title 'FastFly-Adapt' with linespoints linetype 1 pointtype 14 linecolor rgbcolor "#0000FF"

set border
set tics


#=========================================
set size 0.7, 0.5;
set origin 0, 0.47;

set xlabel "POI number" offset 0,0.1,0
set ylabel "Construction Time (s)" offset 0.9,0,0
set key above
set log y

set xrange [45: 255]
set yrange [0.01: 30000] 
set xtics rotate by 45 right
set label 11 center at graph 0.5,char 1 "(a)" offset 0,13.9,0
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("50" 50, "100" 100, "150" 150, "200" 200, "250" 250)

plot "18_SE_Oracle.txt" using 3:($6/1000) notitle with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"19_EAR_Oracle.txt" using 3:($6/1000) notitle with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"20_RC_Oracle_Naive_Adapt.txt" using 3:($6/1000) notitle with linespoints linetype 1 pointtype 6 linecolor rgbcolor "#000000", \
"21_RC_Oracle_Adapt.txt" using 3:($6/1000) notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 0.6, 0.47;

set xlabel "POI number"
set ylabel "Memory (MB)"
set key above
set log y

set xrange [45: 255]
set yrange [0.01: 5000]
set label 11 center at graph 0.5,char 1 "(b)" 
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("50" 50, "100" 100, "150" 150, "200" 200, "250" 250)

plot "18_SE_Oracle.txt" using 3:9 notitle with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"19_EAR_Oracle.txt" using 3:9 notitle with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"20_RC_Oracle_Naive_Adapt.txt" using 3:9 notitle with linespoints linetype 1 pointtype 6 linecolor rgbcolor "#000000", \
"22_CH.txt" using 3:9 notitle with linespoints linetype 1 pointtype 8 linecolor rgbcolor "#808080", \
"23_Kaul.txt" using 3:9 notitle with linespoints linetype 1 pointtype 10 linecolor rgbcolor "#808080", \
"24_Dijk.txt" using 3:9 notitle with linespoints linetype 1 pointtype 12 linecolor rgbcolor "#808080", \
"25_FastFly_Adapt.txt" using 3:9 notitle with linespoints linetype 1 pointtype 14 linecolor rgbcolor "#0000FF", \
"21_RC_Oracle_Adapt.txt" using 3:9 notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 1.2, 0.47;

set xlabel "POI number"
set ylabel "Size (MB)"
set key above
set log y

set xrange [45: 255]
set yrange [0.05: 300]
set label 11 center at graph 0.5,char 1 "(c)" 
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("50" 50, "100" 100, "150" 150, "200" 200, "250" 250)

plot "18_SE_Oracle.txt" using 3:10 notitle with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"19_EAR_Oracle.txt" using 3:10 notitle with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"20_RC_Oracle_Naive_Adapt.txt" using 3:10 notitle with linespoints linetype 1 pointtype 6 linecolor rgbcolor "#000000", \
"21_RC_Oracle_Adapt.txt" using 3:10 notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 1.8, 0.47;

set xlabel "POI number"
set ylabel "Query Time (ms)"
set key above
set log y

set xrange [45: 255]
set yrange [0.0005: 20000]
set label 11 center at graph 0.5,char 1 "(d)" 
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("50" 50, "100" 100, "150" 150, "200" 200, "250" 250)

plot "18_SE_Oracle.txt" using 3:7 notitle with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"19_EAR_Oracle.txt" using 3:7 notitle with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"20_RC_Oracle_Naive_Adapt.txt" using 3:7 notitle with linespoints linetype 1 pointtype 6 linecolor rgbcolor "#000000", \
"22_CH.txt" using 3:7 notitle with linespoints linetype 1 pointtype 8 linecolor rgbcolor "#808080", \
"23_Kaul.txt" using 3:7 notitle with linespoints linetype 1 pointtype 10 linecolor rgbcolor "#808080", \
"24_Dijk.txt" using 3:7 notitle with linespoints linetype 1 pointtype 12 linecolor rgbcolor "#808080", \
"25_FastFly_Adapt.txt" using 3:7 notitle with linespoints linetype 1 pointtype 14 linecolor rgbcolor "#0000FF", \
"21_RC_Oracle_Adapt.txt" using 3:7 notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 0.3, -0.02;

set xlabel "POI number"
set ylabel "kNN Query Time (ms)"
set key above
set log y

set xrange [45: 255]
set yrange [1: 2000000]
set label 11 center at graph 0.5,char 1 "(e)" offset 0,-0.4,0
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("50" 50, "100" 100, "150" 150, "200" 200, "250" 250)

plot "18_SE_Oracle.txt" using 3:($3*$3*$7) notitle with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"19_EAR_Oracle.txt" using 3:($3*$3*$7) notitle with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"20_RC_Oracle_Naive_Adapt.txt" using 3:($3*$3*$7) notitle with linespoints linetype 1 pointtype 6 linecolor rgbcolor "#000000", \
"22_CH.txt" using 3:($3*$7) notitle with linespoints linetype 1 pointtype 8 linecolor rgbcolor "#808080", \
"23_Kaul.txt" using 3:($3*$7) notitle with linespoints linetype 1 pointtype 10 linecolor rgbcolor "#808080", \
"24_Dijk.txt" using 3:($3*$7) notitle with linespoints linetype 1 pointtype 12 linecolor rgbcolor "#808080", \
"25_FastFly_Adapt.txt" using 3:($3*$7) notitle with linespoints linetype 1 pointtype 14 linecolor rgbcolor "#0000FF", \
"21_RC_Oracle_Adapt.txt" using 3:($3*$3*$7) notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 0.9, -0.02;

set xlabel "POI number"
set ylabel "Range Query Time (ms)"
set key above
set log y

set xrange [45: 255]
set yrange [1: 2000000]
set label 11 center at graph 0.5,char 1 "(f)" 
set bmargin 5
set format x "%g"
set format y "10^{%T}"

set xtics ("50" 50, "100" 100, "150" 150, "200" 200, "250" 250)

plot "18_SE_Oracle.txt" using 3:($3*$3*$7) notitle with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"19_EAR_Oracle.txt" using 3:($3*$3*$7) notitle with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"20_RC_Oracle_Naive_Adapt.txt" using 3:($3*$3*$7) notitle with linespoints linetype 1 pointtype 6 linecolor rgbcolor "#000000", \
"22_CH.txt" using 3:($3*$7) notitle with linespoints linetype 1 pointtype 8 linecolor rgbcolor "#808080", \
"23_Kaul.txt" using 3:($3*$7) notitle with linespoints linetype 1 pointtype 10 linecolor rgbcolor "#808080", \
"24_Dijk.txt" using 3:($3*$7) notitle with linespoints linetype 1 pointtype 12 linecolor rgbcolor "#808080", \
"25_FastFly_Adapt.txt" using 3:($3*$7) notitle with linespoints linetype 1 pointtype 14 linecolor rgbcolor "#0000FF", \
"21_RC_Oracle_Adapt.txt" using 3:($3*$3*$7) notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


#=========================================
set size 0.7, 0.5;
set origin 1.5, -0.02;

set xlabel "POI number"
set ylabel "Distance Error"
set key above
unset log y

set xrange [45: 255]
set yrange [0: 0.12]
set label 11 center at graph 0.5,char 1 "(g)" 
set bmargin 5
set format x "%g"
set format y "%g"

set xtics ("50" 50, "100" 100, "150" 150, "200" 200, "250" 250)

plot "18_SE_Oracle.txt" using 3:12 notitle with linespoints linetype 1 pointtype 1 linecolor rgbcolor "#000000", \
"19_EAR_Oracle.txt" using 3:12 notitle with linespoints linetype 1 pointtype 2 linecolor rgbcolor "#000000", \
"20_RC_Oracle_Naive_Adapt.txt" using 3:12 notitle with linespoints linetype 1 pointtype 6 linecolor rgbcolor "#000000", \
"22_CH.txt" using 3:12 notitle with linespoints linetype 1 pointtype 8 linecolor rgbcolor "#808080", \
"23_Kaul.txt" using 3:12 notitle with linespoints linetype 1 pointtype 10 linecolor rgbcolor "#808080", \
"24_Dijk.txt" using 3:12 notitle with linespoints linetype 1 pointtype 12 linecolor rgbcolor "#808080", \
"25_FastFly_Adapt.txt" using 3:12 notitle with linespoints linetype 1 pointtype 14 linecolor rgbcolor "#0000FF", \
"21_RC_Oracle_Adapt.txt" using 3:12 notitle with linespoints linetype 1 pointtype 4 linecolor rgbcolor "#FF0000"


unset multiplot
