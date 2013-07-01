#!/bin/csh
# This assumes the edm environment is set up, and we are passed two arguments: EVR and IOC.
set all = (0 1 2 3 4 5 6 7 8 9 A B)
set cnt = 0
set ours = "EVR=$1,IOC=$2"
set off = ""
foreach i ($all)
    if (X`caget -nt ${1}:CTRL.IP${i}E` == X1) then
	set ours = $ours,N${cnt}=$i
        @ cnt = $cnt + 1
    else
        set off = $i
    endif
end
if ($cnt <= 4) then
    set use = 4
else
    if ($cnt <= 8) then
        set use = 8
    else
        set use = 12
    endif
endif
while ($cnt < $use)
    set ours = $ours,N${cnt}=$off
    @ cnt = $cnt + 1
end
edm -x -eolc -m $ours evrscreens/evr$use.edl >& /dev/null
