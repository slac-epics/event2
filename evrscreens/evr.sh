#!/bin/csh
# This assumes the edm environment is set up, and we are passed two arguments: EVR and IOC.
set evr = $1
set ioc = $2
set all = (0 1 2 3 4 5 6 7 8 9 A B)
set use = (1 2 3 4 8 8 8 8 12 12 12 12)
set cnt = 0
if (X$ioc == X) then
    set plist = "EVR=$evr"
else
    set plist = "EVR=$evr,IOC=$ioc"
endif
set off = ""
foreach i ($all)
    if (X`caget -nt ${evr}:CTRL.IP${i}E` == X1) then
	set plist = $plist,N${cnt}=$i
        @ cnt = $cnt + 1
    else
        set off = $i
    endif
end
@ u = $use[$cnt]
while ($cnt < $u)
    set plist = $plist,N${cnt}=$off
    @ cnt = $cnt + 1
end
edm -x -eolc -m $plist evrscreens/evr$u.edl >& /dev/null
