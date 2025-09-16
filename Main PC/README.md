# Main PC Documentation

## Dependencies

#### Python 3.6+ (pip, numpy, pyserial, termcolor, numba, posix_ipc, lcm)

## Instructions

### Serial Port & MAC Address
Please config the serial port names of BEARs and IMU, MAC address of your gamepad in ``THEMIS/THEMIS_SERIAL_PORT`` and ``THEMIS/THEMIS/Settings/Macros/constant_macros.py`` for your THEMIS before use.

You can get the serial port names by the following command in terminal:
```bash
ls /dev/serial/by-id/
```

### Allow Executing Bash/Binary File As Program
1. Download the binary [controller](https://dynabotic.feishu.cn/file/NcP5bdVEXoIAZtxn5M0cUfminCd) and place the unzipped files under ``THEMIS/THEMIS/Play/Locomotion/``.
2. Go to THEMIS folder.
    ```bash
    cd THEMIS/THEMIS
    ```
3. Run the following commands.
    ```bash
    chmod +x Play/bootup.sh
    chmod +x Play/bootup_deck.sh
    chmod +x Play/sim_bootup.sh
    chmod +x Play/terminate.sh
    chmod +x Play/terminate_chair.sh
    chmod +x Play/Locomotion/low_level
    chmod +x Play/Locomotion/high_level
    chmod +x Simulation/launch_gazebo.sh
    chmod +x Startup/startup_setup.sh
    chmod +x Startup/usb_latency_setup.sh
    chmod +x Util/poweroff.sh
    ```

### Startup Setup Before Launch
1. Go to THEMIS folder.
    ```bash
    cd THEMIS/THEMIS
    ```
2. Config launch setup in ``Play/config.py``.
3. Run the bash file.
    ```bash
    Startup/startup_setup.sh
    ```

### Operating
1. Go to THEMIS folder.
    ```bash
    cd THEMIS/THEMIS
    ```
2. Run the bash file and follow the guidance.
    ```bash
    Play/bootup.sh
    ```
3. Terminate in the end.
    ```bash
    Play/terminate.sh
    ```

### Simulation (THEMIS Gym)
THEMIS Gym simulation is based on a custom library to interact with Gazebo. Please refer to [README](https://github.com/Westwood-Robotics/THEMIS-OP/blob/main/Main%20Computer/THEMIS/THEMIS-OP/Simulation/README.md) in the Simulation folder for detailed instructions.