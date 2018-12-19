0.  Test Prerequisites
Need functioning EVR w/ valid timing signal
Select a trigger number and set EVR, TRIG, and TRIG_CH env vars
Example for trigger number 2:
EVR=$TST:EVR:EDT:PTM1
TRIG=$EVR:TRIG2
TRIG_CH=2

1.  Set Delay via TDES
Setup: Set $TRIG:TEC to 41, $TRIG:TDES to 10000.
Verify:
	$EVR:CTRL.DG${TRIG_CH}D = 76
	$TRIG:BW_TDLY = 76
	$TRIG:BW_TDES = 10000.00 ns +- 5
	$TRIG:EC_RBV  = 41

2.	Change Event code to 40
Setup: Set $TRIG:TEC to 40
Verify:
	$TRIG:TDES still = 10000.0 ns
	$EVR:CTRL.DG${TRIG_CH}D = 86
	$TRIG:BW_TDLY = 86
	$TRIG:BW_TDES = 10000.00 ns +- 5
	$TRIG:EC_RBV  = 40

3.	Change Event code to 42
Setup: Set $TRIG:TEC to 42
Verify:
	$TRIG:TDES still = 10000.0 ns
	$EVR:CTRL.DG${TRIG_CH}D = 66
	$TRIG:BW_TDLY = 66
	$TRIG:BW_TDES = 10000.00 ns +- 5
	$TRIG:EC_RBV  = 40

4.	Restart IOC either via iocsh exit command, $IOC:SYSRESET, or IOC screen Reboot command.
Setup: Set $IOC:SYSRESET to 1
Verify:
	$TRIG:TDES still = 10000.0 ns
	$TRIG:TEC = 40
	$EVR:CTRL.DG${TRIG_CH}D = 66
	$TRIG:BW_TDLY = 66
	$TRIG:BW_TDES = 10000.00 ns +- 5
	$TRIG:EC_RBV  = 40

5.	Test selection of event codes which occur before TDES
Setup:
	Set $TRIG:TEC to 42
	Set $TRIG:TDES to 9600 ns
Verify:
	$EVR:CTRL.DG${TRIG_CH}D = 18
	$TRIG:TDES = 9600.0 ns
	$TRIG:BW_TDES = 9600.00 ns +- 5
	$TRIG:BW_TDLY = 18
	$TRIG:EC_RBV  = 42
Test event code 44:
	Set $TRIG:TEC to 44
Verify:
	$EVR:CTRL.DG${TRIG_CH}D = 18
	$TRIG:TDES = 9600.0 ns
	$TRIG:BW_TDES = 9764.7 ns +- 5
	$TRIG:BW_TDLY = 18
	$TRIG:EC_RBV  = 44
