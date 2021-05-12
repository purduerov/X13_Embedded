# X13-Embedded

Embedded code for the ESCs, solenoids, bricks, and more using CAN communication.

This code is developed with [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
for the [STM32F042K6T6](https://www.st.com/content/ccc/resource/technical/document/datasheet/52/ad/d0/80/e6/be/40/ad/DM00105814.pdf/files/DM00105814.pdf/jcr:content/translations/en.DM00105814.pdf).
See the [F0 reference manual](https://www.st.com/resource/en/reference_manual/dm00031936-stm32f0x1stm32f0x2stm32f0x8-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) too.


## Building and generating code
Driver code is not tracked by git. To get it, open the IOC file for the project
and go to Project > Generate Code. This will produce the appropriate Driver/ folders
for the code.

### Recommended settings
##### Optimization
Set optimization for Release builds to -03 rather than -Osize (the default). This setting is in
Project > Properties > C/C++ Build > Settings > Tool Settings > MCU GCC Compiler > Optimization.
This should only be done for Release builds. Debug builds should not be optimization

##### Whitespace
In Window > Preferences > C/C++ > Editor > Save Actions
* Check "Remove trailing whitespace"
* Select the "In all lines"
* Check "Ensure newline at the end of file"

This makes it easier to diff files in git and prevent unnecessary changes.
