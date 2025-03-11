# Sensor Node hardware and software for GR25

---

## Hardware 
The Hardware is split into a couple of folders and have Kicad projects that relate to the Sensor Nodes. This includes random projects and some hardware that might have been usefull.

---

## Software

Includes software for the Sensor Nodes

### Compilation:

To compile and be able to debug the project you need the following: 
- CMAKE 
- arm-none-eabi-gcc
- openocd
- ninja
- VSCode CMAKE extension
- VSCode Cortex Debug extension

**Make sure cmake, arm tolchian, openocd,and ninja are added to path**

To check the version run: 
- cmake --version
- arm-none-eabi-gcc --version
- openocd --version
- ninja --version

After everything is installed you should be able to follow the VSCode UI to compile and debug.


