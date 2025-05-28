# AstriForce - Uniaxial Tension + Compression Testing
## Empowering small labs and startups with hackable aerospace-grade mechanical test capability.

## Overview
This project combines a microcontroller-based system with a Python GUI to control and collect data from a tensile testing machine. It allows for real-time position or load-based control from GUI inputs or Python scripts with simultaneous data acquisition and visualization.

## Features
- Real-time Control: Adjust motor and heater settings through a user-friendly GUI.
- Data Acquisition: Collect sensor data (load, displacement) in real-time.
- Data Logging: Save data continuously in simple, human-readable formats.
- Graphical Interface: Visualize sensor data with matplotlib, enable core functionality via user interface.

## Project Components
### Microcontroller Software (Arduino C++ via PlatformIO)
- Functionality:
  - Control Task: Manages the hardware interface including motor and heater controls.
  - Sensor Task: Reads and calculates mean values from thermocouples (TCs).
  - Serial Port Task: Manages communication between the GUI and microcontroller, parsing JSON inputs and sending sensor data.
- Hardware: Teensy 4.1
### GUI Software (Python)
- Dependencies:
  - Python 3.x
  - tkinter for GUI
  - matplotlib for plotting
  - pyserial for USB communication
  - pandas for data handling
- Functionality:
  - Serial Communication: Establishes connection over USB, reads and writes data.
  - User Interface: Provides controls for setting operational parameters, initiating data collection, and viewing real-time plots.
  - Data Management: Saves collected data in CSV format and logs operations in plain text.

## Hardware Setup
- Code is compatible with the AstriForce electrical schematic (TODO - link to schematics)

## Software Setup
### Microcontroller:
1. Install Visual Studio Code (VSCode) if not already installed.
2. Install PlatformIO Extension:
   - Open VSCode, go to the Extensions view (Ctrl+Shift+X), search for "PlatformIO IDE", and install it.
3. Configure PlatformIO:
   - Create a new project by selecting PlatformIO Home from the PlatformIO menu in VSCode, then choose New Project.
   - Select Teensy4.1 as the board, Arduino as the framework, and name your project.
   - Place main.cpp into the src directory of your new project.
4. Build and Upload:
   - Use the PlatformIO menu in VSCode, click on Upload to compile and upload main.cpp to your ESP32.

### GUI:
1. Install Python if not already installed.
2. Install required libraries with pip: pip install tkinter matplotlib pyserial pandas
3. Run the Python Script: python SimpleDAQ.py

### Usage
1. Start the GUI by running the Python script.
2. Type in the correct COM port for the microcontroller device
3. Start data acquisition and control
4. Exit at any point to save data

### Contribute
1. Fork the repository.
2. Make your changes or additions.
3. Open a Pull Request with a clear description of what your changes do.

### Software License
This software is licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at
[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)
Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.
**Credit Requirement:** Please retain and include the copyright notice and any NOTICE file content in all copies or derivative works of this software. 

### Hardware License
The hardware designs of this project are licensed under the CERN Open Hardware License Version 2 - Strongly Reciprocal (CERN-OHL-S v2). 
You can read the full license text here: [CERN-OHL-S v2](https://ohwr.org/project/cernohl/wikis/Documents/CERN-OHL-S_v2).
**Credit Requirement:** Any derivative work must acknowledge the original source by including a notice that the original design was created by Astris Design.

### Development Status
This project is currently under active development. 

### Acknowledgements
Thanks to TJ Parekh for contributing to early mechanical design!

### Contact
For questions, issues, or suggestions, please email clayton@astrisdesign.com or open an issue in the repository.
