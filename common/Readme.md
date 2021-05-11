# Common library for embedded

This is meant to a directory for common files that can be shared across projects.

Included are:
* common.h - A header file for useful macros and typedefs
* canFilter
* queue


### Setup
To add the common directory to your project:
Go to Project > Properties > C/C++ Build > Settings > Tool Settings > MCU GCC Compiler > Include Paths
and add "../../common" to the list of paths.

The neccesary source files also need to be compiled.  To specify this:
Go to Project > Properties > C/C++ General > Paths and Symbols > Source Location
Click the "Link Folder" button
Check "Link to folder in file system"
Browse for "common" in repository directory

*Optional Steps to avoid compiling unused files*
Go to Project > Properties > C/C++ General > Paths and Symbols > Source Location
Select "/<project name>/common"
Click the "Edit Filter" button
Add files to exclude by clicking the "Add" button



For the `DEBUG`/`NDEBUG` macros, you should add `NDEBUG` to be used for release builds. To do this, go to
Project > Properties > C/C++ Build > Settings > Tool Settings > MCU GCC Compiler > Preprocessor
Pick "Release" from the dropdown at the top and add `NDEBUG`.

