# General Info

This is a guide on how to use Windows MSYS2 distribution and building platform to build iNav firmware. This environment is very simple to manage and does not require installing docker for Windows which may get in the way of VMWare or any other virtualization software you already have running for other reasons. Another benefit of this approach is that the compiler runs natively on Windows, so performance is much better than compiling in a virtual environment or a container. You can also integrate with whatever IDE you are using to make code edits and work with github, which makes the entire development and testing workflow a lot more efficient. In addition to MSYS2, this build environment also uses Arm Embedded GCC tolkit from The xPack Project, which provides many benefits over the toolkits maintained by arm.com

Some of those benefits are described here:

https://xpack.github.io/arm-none-eabi-gcc/

## Setting up build environment

Download MSYS2 for your architecture (most likely 64-bit)

https://www.msys2.org/wiki/MSYS2-installation/

Click on 64-bit, scroll all the way down for the latest release

pacman is the package manager which makes it a lot easier to install and maintain all the dependencies

## Installing dependencies

Once MSYS2 is installed, you can open up a new terminal window by running:

"C:\msys64\mingw64.exe"

You can also make a shortcut of this somewhere on your taskbar, desktop, etc, or even setup a shortcut key to open it up every time you need to get a terminal window. If you right click on the window you can customize the font and layout to make it more comfortable to work with. This is very similar to cygwin or any other terminal program you might have used before

This is the best part:
```
pacman -S git ruby make cmake gcc mingw-w64-x86_64-libwinpthread-git unzip wget
```

Now, each release needs a different version of arm toolchain. To setup the xPack ARM toolchain, use the following process:

First, setup your work path, get the release you want to build or master if you want the latest/greatest
```
mkdir /c/Workspace
cd /c/Workspace
# you can also check out your own fork here which makes contributing easier
git clone https://github.com/iNavFlight/inav
cd inav
# switch to release you want or skip next 2 lines if you want latest
git fetch origin
git checkout -b release_2.6.1 origin/release_2.6.1
```
Now create the build and xpack directories and get the toolkit version you need for your inav version
```
mkdir build
cd build
mkdir /c/Workspace/xpack
cd /c/Workspace/xpack
cat /c/Workspace/inav/cmake/arm-none-eabi-checks.cmake | grep "set(arm_none_eabi_gcc_version" | cut -d\" -f2
```
This will give you the version you need for any given release or master branch. You can get to all the releases here and find the version you need

https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/
```
# for version 2.6.1, version needed is 9.2.1
wget https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v9.2.1-1.1/xpack-arm-none-eabi-gcc-9.2.1-1.1-win32-x64.zip
unzip xpack-arm-none-eabi-gcc-9.2.1-1.1-win32-x64.zip
```
This is important, put the toolkit first before your path so that it is  picked up ahead of any other versions that may be present on your system
```
export PATH=/c/Workspace/xpack/xpack-arm-none-eabi-gcc-9.2.1-1.1/bin:$PATH
cd /c/Workspace/inav/build
```
You may need to run rm -rf * in build directory if you had any failed previous runs now run cmake
```
# while inside the build directory
cmake ..
```
Once that's done you can compile the firmware for your controller
```
make DALRCF405
```
To get a list of available targets in iNav, see the target src folder
https://github.com/tednv/inav/tree/master/src/main/target

The generated hex file will be in /c/Workspace/inav/build folder

At the time of writting this document, I believe this is the fastest, easiest, and most efficient Windows build environment that is available. I have used this approach several years ago and was very happy with it building iNav 2.1 and 2.2, and now I'm getting back into it so figured I would share my method
