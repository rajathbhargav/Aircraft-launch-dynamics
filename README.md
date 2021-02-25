# Aircraft-launch-dynamics
Simulate take-off characteristics for a propeller driven aircraft

"aero-hand.py" - This code simulates the take-off dynamics of a propreller-driven **hand-launched** aircraft.
By default the loop is rigged to iterate through a range of thrust values and to break at the least thrust required to achieve an un-eventful launch.
The loop can be modified to iterate for any other parameter and optimize it.


"aero-run.py" - This code simulates the take-off dynamics of a propreller-driven aircraft on a **runway**.
By default the loop is rigged to iterate through a range of thrust values and to break at the least thrust required to take off within runway limits.
The loop can be modified to iterate for any other parameter and optimize it.

Note: 
The numbers are an approximation as a result of some ignored factors and finite data precision. Should be used as a good starting point for any design and to observe trends.
