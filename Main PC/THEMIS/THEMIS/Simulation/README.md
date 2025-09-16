# THEMIS Gym

THEMIS Gym is the simulation environment for THEMIS. It utilizes a custom library to interact THEMIS in Gazebo using python.

## Dependencies

#### Python 3.6+ (pip, numpy, termcolor, numba, posix_ipc)

## Installation

#### 1. [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)
The latest version, Gazebo11, is recommended.

#### 2. Link THEMIS model to Gazebo
```bash
cd THEMIS-OP
mkdir ~/.gazebo/models
ln -s $PWD/Simulation/models/themis ~/.gazebo/models/themis
```

#### 3. Add THEMIS Gym plugins to Gazebo
```bash
cd THEMIS-OP
cp -r Library/THEMIS_GYM/GAZEBO_PLUGIN ~/.gazebo
echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/.gazebo/GAZEBO_PLUGIN"  >>  ~/.bashrc
source ~/.bashrc
```

## Instructions

### Indicating Operating in Simulation
Modify line 15 of ``Play/config.py``.
```python
SIMULATION = True
```

### Indicating Floating Base or Fixed Base
Modify line 15 of ``Simulation/config.py``.
```python
FIXED = False
```

### Loading THEMIS model in Gazebo
```bash
cd THEMIS-OP
Simulation/launch_gazebo.sh
```
If everything goes well, THEMIS should be in the air in its nominal posture.

_ATTENTION:_
THEMIS Gym is built on Ubuntu 22.04.5 x86-64. Any lower version might need an upgrade of the GNU C libraries, e.g., GLIBC and GLIBCXX. Please refer to the error messages in this regard.

### Operating
1. Go to THEMIS folder.
    ```bash
    cd THEMIS
    ```
2. Config launch setup in ``Play/config.py``.
3. Run the bash file and follow the guidance.
    ```bash
    Play/sim_bootup.sh
    ```

### Got Errors?
1. There is probably another Gazebo process running ...
 ```bash
killall gzserver
killall gzclient
 ```

### Other Notes
To quickly visualize your URDF online: [HERE](https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/index.html)