# Common library for embedded

This is meant to a directory for common files that can be shared across projects.

Included are:
* common.h - A header file for useful macros and typedefs
* canFilter - Configuration functions for CAN filters
* queue - A generic, robust Queue implementation


### Setup
To add the common directory to your project:
Go to Project > Properties > C/C++ Build > Settings > Tool Settings > MCU GCC Compiler > Include Paths
and add "../../common" to the list of paths. **Note**: this must be done for Debug and Release.

The neccesary source files also need to be compiled.  To specify this:
Go to Project > Properties > C/C++ General > Paths and Symbols > Source Location.
Click the "Link Folder" button.
Enter "common" (without the quotes) into the Folder name box.
Check "Link to folder in file system".
Enter "PROJECT_LOC/../common" in the box. PROJECT_LOC is a variable for the path to
the current project.


*Optional Steps to avoid compiling unused files*
Go to Project > Properties > C/C++ General > Paths and Symbols > Source Location
Select "/<project name>/common"
Click the "Edit Filter" button
Add files to exclude by clicking the "Add" button



For the `DEBUG`/`NDEBUG` macros, you should add `NDEBUG` to be used for release builds. To do this, go to
Project > Properties > C/C++ Build > Settings > Tool Settings > MCU GCC Compiler > Preprocessor
Pick "Release" from the dropdown at the top and add `NDEBUG`.

