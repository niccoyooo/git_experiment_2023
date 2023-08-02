Added MavLogging.py -> Uses pymavutil
There are other options such as dronekit to explore sending mavlink messages

Chose to use pyMavUtil, updated code to use RC_OVERRIDE for logging (check RCIN on Mission Planner Logs)
Code can now take in voltage from the string pots, and convert it to an integer suitable to be loggeed (between 1000 and 2000)
