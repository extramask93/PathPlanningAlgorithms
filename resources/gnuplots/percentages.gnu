
# set terminal pngcairo  transparent enhanced font "arial,10" fontscale 1.0 size 600, 400 
# set output 'simple.8.png'
set key bmargin left horizontal Right noreverse enhanced autotitle box lt black linewidth 1.000 dashtype solid
set style increment default
set samples 800, 800
set title  font ",20" norotate
set xrange [ * : * ] noreverse writeback
set x2range [ * : * ] noreverse writeback
set yrange [ * : * ] noreverse writeback
set y2range [ * : * ] noreverse writeback
set zrange [ * : * ] noreverse writeback
set cbrange [ * : * ] noreverse writeback
set rrange [ * : * ] noreverse writeback
set grid xtics ytics
set ylabel "Czas wyszukiwania [%]"
set format y '%g'
set logscale y
set xlabel "Algorytm"

set notitle
set boxwidth 0.5
set style fill solid
set terminal qt 0
set xtics ("A*" 1, "Dijkstra" 2, "DFS" 3, "RRT" 4, "RRT*" 5,"PRM" 6, "ACO" 7)
stats 'percentages.dat' using 2
plot 'percentages.dat' using 1:(($2/STATS_min)*100 -100) w boxes notitle
set terminal qt 1
unset logscale y
stats 'percentages.dat' using 3
set ylabel "Dlugosc sciezki [%]"
plot 'percentages.dat' using 1:(($3/STATS_min)*100-100) w boxes notitle
