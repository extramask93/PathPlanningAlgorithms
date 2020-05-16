
# set terminal pngcairo  transparent enhanced font "arial,10" fontscale 1.0 size 600, 400 
# set output 'simple.8.png'
set key bmargin left horizontal Right noreverse enhanced autotitle box lt black linewidth 1.000 dashtype solid
set style increment default
set samples 800, 800
set title "Srednia dlugosc trasy"
set title  font ",20" norotate
set xrange [ * : * ] noreverse writeback
set x2range [ * : * ] noreverse writeback
set yrange [ * : * ] noreverse writeback
set y2range [ * : * ] noreverse writeback
set zrange [ * : * ] noreverse writeback
set cbrange [ * : * ] noreverse writeback
set rrange [ * : * ] noreverse writeback
set grid xtics ytics
set multiplot layout 2,1
set xlabel "Numer punktu" font ",10"
set ylabel "Dlugosc trasy [m]"

set title "Dlugosc trasy" font ",15"
plot [1:7]'gamma.dat' using 1:3 every 5::0 w lp title "{/Symbol g} = 5" pt 7 ps 2, '' using 1:3 every 5::1 w lp title "{/Symbol g}=10" pt 7 ps 2, \
 '' using 1:3 every 5::2 w lp title "{/Symbol g}=20" pt 7 ps 2, '' using 1:3 every 5::3 w lp title "{/Symbol g}=50" pt 7 ps 2, \
 '' using 1:3 every 5::4 w lp title "{/Symbol g}=100" pt 7 ps 2


set title "Czas wyszukiwania" font ",15"
set ylabel "Czas wyszukiwania [ms]"
plot [1:7]'gamma.dat' using 1:4 every 5::0 w lp title "{/Symbol g} = 5" pt 7 ps 2, '' using 1:4 every 5::1 w lp title "{/Symbol g}=10" pt 7 ps 2, \
 '' using 1:4 every 5::2 w lp title "{/Symbol g}=20" pt 7 ps 2, '' using 1:4 every 5::3 w lp title "{/Symbol g}=50" pt 7 ps 2, \
 '' using 1:4 every 5::4 w lp title "{/Symbol g}=100" pt 7 ps 2



unset multiplot
