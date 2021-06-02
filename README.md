# SVEA Low Level Interface (LLI)
This repository contains firmware for the SVEA Powerboard 2.
PlatformIO is used to build and upload the firmware. 
Instructions and scripts for installing PlatformIO 
together with other dependencies are included in this repository.
See the [Setup](#setup) section for further instructions. 
## Repository structure
- [Source code location](src/)

	This folder contains the source code for the firmware

- [Libraries](lib/)

	Contains libraries

- [Target definition](targets/)

	Contains a 

- [Build scripts and other utilities](resources/)
- [PlatformIO configurations](/platformio.ini)
- [Developer instructions](docs/DEVELOPMENT.md)

## Setup
There are three options for how to install platformIO.
The two first options will create a virtual environment for PlatformIO.
The last option will install the PlatformIO library directly with
your other python libraries.

### For development with Visual Code
_Recommended for development_
1. `git clone` this repository to a folder on your computer
2. [If a conda environemnt is acitve] Deactivate any active conda environment with `conda deactivate`
3. Install the platformIO plugin for VS Code as described in https://platformio.org/install/ide?install=vscode
4. Go to the root folder of this repository and run `bash init.sh`
5. Open the reository folder in VS Code

### Command line only
_Recommended for just flashing new firmware_
1. `git clone` this repository to a folder on your computer
2. [If a conda environemnt is acitve] Deactivate any active conda environment with `conda deactivate`
3. Install PlatformIO by executing: `python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)"`
4. Go to the root folder of this repository and run `bash init.sh`

### I'm in a hurry and I just need to flash this Powerboard NOW
_Not recommended unless the previous methods has already failed_
1. `git clone` this repository to a folder on your computer.
2. Go to the root folder of this repository and run `bash init.sh`

## Usage
### Compile and upload firmware
#### From command line
1. Navigate to the root folder of the repository
2. Actiate the PlatformIO vritual environment with `source ~/.platformio/penv/bin/activate`
3. Connect the Powerboard via USB
4. Execute `pio run -t upload`

#### From VS Code
VS Code can be used to build and upload the firmware to the Powerboard if the PlatformIO plugin is installed. 
1. Open the root folder of this repository in VS Code
2. Go to the _PlatformIO_ tab in VS code
3. Klick on the _Default_ folder
4. Open the _General_ tab
5. Klick on _Upload All_ (Only one environemnt will actually be built and uploaded) 
The virtual environment will automatically be activated by the PlatformIO plugin.

### Available environments 
There are 5 different environemnts defined in [platformio.ini](platformio.ini). 
The difference between these environemnts is how the rosserial libraries are created.
The environment can be set with the `-e` flag, e. g. `-e *environment name*`.

#### use_included_libs
The default environement. 
Uses the rosserial libraries included in [lib/ros_included](lib/ros_included). 
This is the environment that runs if no 
`--environment` or `-e` argument is passed to `pio run`. 
This environment is can be used if no messages defintions have been added or changed.
Note that most of the standard message defitions,
like `std_msgs`, `geometry_msgs`, and `nav_msgs` are already included.

#### build_local_libs
Build the rosserial libraries from the currently sourced ROS workspaces.
Rosserial and all required message definitions have to be included
in these workspaces, or the compilation will fail.
Intended to be used for development where messages have been added
or altered on the current machine.
The newly built rosserial libraries will be placed in a folder named
`lib/ros_local`.
__Note: This environment generates rosserial messages, but does not build the workspaces.__
__The workspaces will have to be built manually for any changes to message definitions to take effect.__

#### use_local_libs
Same as `use_included_libs` but will use
rosserial libraries located in [lib/ros_local](lib/ros_local) 
(previously generated by `build_local_libs`), instead of [lib/ros_included](lib/ros_included).

#### download_and_build_remote_libs
Download both rosserial and all messages from the `vehicle_msgs` repository, 
build them and place the rosserial library in [lib/ros_remote](lib/ros_remote).
Some temporary ROS workspaces will be placed in a _tmp_ folder in the root of the repository.

#### use_remote_libs
Same as `use_included_libs` but will use
rosserial libraries located in [lib/ros_remote](lib/ros_remote) instead
(previously generated by `download_and_build_remote_libs`), instead of [lib/ros_included](lib/ros_included).


## FAQ

<details>
<summary>
The ROS header file cannot be found.
</summary>


Ensure that you ran `init.sh` (required to use automated hooks) and that you restarted your login session. (reboot or re-login)
You can test if `echo ${tmw_DIR}` outputs the directory of your `teensy_firmware` clone.

Also ensure that you have a `catkin` compatible Python version. (Note that Python 3.7 has a trollius and async issue. Easiest is to switch your virtualenvironment to Python2 and install the dependencies with `pip install -r requirements.txt`. Also ensure that after the virtual environment switch you should execute `source /opt/ros/${ROS_DISTRO}/setup.{sh,zsh}`).
</details>


<details>
<summary>
The build fails and throws some error.
</summary>


Inspect the logs generated by the hooks under `./firmware/log/middleware.log`


The output could look similar to the following snippet.
```bash
(vp2) firmware cat log/middleware.log
Input arguments received:
	 template git clone https://github.com/prothen/testbed_msgs.git
Set project name: template
Parse repository:
	 git clone https://github.com/prothen/testbed_msgs.git
Received additional libraries.
ROS1 interface chosen. Configuring dependencies ... (tzz its 2021...)
BUILD rosserial arduino from upstream:
remote: Enumerating objects: 467, done.
remote: Counting objects: 100% (467/467), done.
remote: Compressing objects: 100% (351/351), done.
remote: Total 467 (delta 115), reused 297 (delta 60), pack-reused 0
Receiving objects: 100% (467/467), 299.88 KiB | 3.57 MiB/s, done.
Resolving deltas: 100% (115/115), done.
----------------------------------------------------------------
```
</details>

## Contribution
Any contribution is welcome.
If you find missing instructions or something did not work as expected please create an issue and let me know.

## License
See the `LICENSE` file for details of the available open source licensing.
