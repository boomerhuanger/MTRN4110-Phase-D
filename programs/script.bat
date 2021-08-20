ECHO Phase C
call activate mtrn4110
PAUSE
python PhaseD_Motion_Tracking_Python.py
PAUSE
:: Webots Simulation
ECHO Phase B
webots z5205551_MTRN4110_PhaseB.v20200620/z5205551_MTRN4110_PhaseB/worlds/z5205551_MTRN4110_PhaseB.wbt
PAUSE
