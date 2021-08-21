ECHO Phase C
call conda activate mtrn4110
::PAUSE
python programs/PhaseD_Motion_Tracking_Python.py
PAUSE
:: Webots Simulation
ECHO Phase B
webots worlds/MTRN4110_PhaseD.wbt
PAUSE
call conda deactivate