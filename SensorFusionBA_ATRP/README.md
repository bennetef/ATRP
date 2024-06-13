# AT-RP Remote Controller

## Execute software
To use this software, Julia V1.7 or newer is required. 

Start the program from REPL: 
1. Start Julia REPL in folder containing the repository
2. Go into package manager by pressing `]`
3. Activate project environment `activate .`
4. Download and compile dependencies `instantiate`
5. Leave package manager
6. Run `julia --project src/Main.jl`
7. Optionally: Can set GUI scale with `julia --project src/Main.jl <scale>`

## How to use the software
The menu bar on the top of the GUI is the main interaction point with the software. Here the __Help__ button
opens a small window to show basic controls of _Dear ImGui_ and for controlling the robot. 

The connect-window lets you connect to the Jetson Nano by entering its port and IP address. If you enter no specific one
the default one will be used, which is used in the Python program controlling the robot.

Once you are connected new menus appear in the menu bar. The __Render Robot__ button toggles the rendering of the robot in
its current orientation. 
There is also the possibility to send automatic commands to the robot. The dialog for that lets you chain commands together
which will be send to the robot when confirming.

### Before connecting to AT-RP
The python program on the robot itself has to be executed before a connection is possible. This program does not start automatically upon activating the Jetson Nano, but needs to be executed manually. 

**Step-by-step walkthrough:**
1. Activate the Jetson Nano
    - This is done by turning the key on the left side of the robot
2. Connect to the WiFi hotspot
    - It is an open network with the name `Jetson`
3. Start the python program
    - There are multiple ways to do this. An easy way to do this, is to connect via an SSH client such as `PuTTY` with the operating system of the robot. The IP would be *10.42.0.1* with Port: *22*. The default username is `at-sv` with `atsv` as the password. The program can then be started by `python3 ~/Documents/Code_AT-RP/atrp_remote_control.py`
4. Connect to the robot through the connection dialog
    - Upon successful connection, more options appear, and sensor data can be displayed. 
5. Before being able to control the robot, the motor and steering servo have to be activated
    - This is done by flipping the switch that is attached on the servo housing
6. Please note that the velocity setting ranges from 19 to 40
    - 19 might be not enough to move the robot!

