**NOTE:** LEDTest program meant for STM32F042K6T6

Instructions for executing LED Test Program:

Configure LEDTest parameters by changing the #define's towards the top of Core/Src/main.c
These parameters are:
LED GPIO Pin Port (A, B, C, or F)
LED GPIO Pin Number [0, 15] (Number between 0 and 15, inclusive)
LED Flash Toggle Enabled (0 -> Disabled, 1 -> Enabled)
LED Flash Toggle Period (Period in milliseconds)

Program Semantics:
At the start of the program, the GPIO output will manually toggle a couple times for
the user "step over" using the debugger.  After this point, the GPIO output will use a timer
to toggle at the specific toggle period, if enabled, for the rest of the program.

Example Parameters:
LED GPIO Pin Port = A
LED GPIO Pin Number = 15
LED Flash Toggle Enabled (0 -> Disabled, 1 -> Enabled)
LED Flash Toggle Period (Period in milliseconds of toggling)
Semantics: GPIO Pin PA15 toggles state every 500ms
