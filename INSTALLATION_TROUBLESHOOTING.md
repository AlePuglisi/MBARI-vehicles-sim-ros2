# Installation process for Dummies and Troubleshooting

## Installation

Step-by-Step installation process and commands to avoid forgetting dependencies...<br/>

After installing it several times (beause of nvidia drivers issues destroying my ubuntu boot)<br/>
I preffer to report all the installation steps for future usage (even the most obvious, considering a fresh linux system)

Because of the previous Nvidia driver issues, I installed the Ubuntu OS without third party drivers, and then add the drivers manually following this [tutorial by Nvidia](https://docs.nvidia.com/datacenter/tesla/driver-installation-guide/index.html#) 

This steps works for me in an `Ubnutu 24.04` OS, with a `6.14.0-33-generic` linux kernel. <br/>

( Dell PRO Max 16 Laptop, NVIDIA RTX PRO 500 Blackwell Generation Laptop GPU, Intel Core Ultra 9, <br/> 
x86_64 (AMD 64bit), in an `Ubnutu 24.04` OS partition, with a `6.14.0-33-generic` linux kernel)

### First, building tools (Dummy reminder N.1)

Always remeber to update and upgrade when installing new software 
```
sudo apt update
```
```
sudo apt upgrade
```
Install building tools
```
sudo apt install make 
```

### OpenGL Mathematics 

```
sudo apt install libglm-dev
```

### SDL 2
From [SDL2 wiki](https://wiki.libsdl.org/SDL2/Installation)

```
git clone https://github.com/libsdl-org/SDL.git -b SDL2
cd SDL
mkdir build
cd build
../configure
cmake ..
make -j$(nproc)
sudo make install
```

> [!NOTE]
> I launch make with -j$(nproc) to use all cpu processors, use -jX to rely on X processors


### FreeType
```
sudo apt install libfreetype6-dev
```
Don't worry if `libfreetype-dev` is installed instead of `libfreetype6-dev`


### Finally, Stonefish
From official [Stonefish documentation](https://stonefish.readthedocs.io/en/latest/install.html)
```
git clone https://github.com/patrykcieslak/stonefish.git
cd stonefish
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install
```

## Troubleshooting

List of problems encountered during the installation process and running tests. <br/>
Problems with uncorrect library installation, linker dependencies, CMakeLists, and nvidia-driver compatibility.

### White Sensors Screen

This is related to the usage of the GPU with the **Mesa Library**. </br>
I previously encountered this issue, but after setting up properly 
- Nvidia Driver 
- OpenGL usage of the Nvidia GPU
  
This code changes were no more required. 

This problem compromise the visualization of sensors output on the Stonefish GUI, seeing white screen instead. <br/>
I found the solution to this bug on [Issue#65](https://github.com/patrykcieslak/stonefish/issues/65#issue-3390696136) of the official stonefish repo. A PR is already open.




