# 🤖 Command-Based Robot Code

"Command-Based" is a way of, in the FIRST context, talk about a ```Object Oriented Programming``` way of setting up the robot code, which is a computer programming model that organizes software design around data, or objects, rather than functions and logic.

<br>

---
# 👨‍💻 Code's Organization 

The code, which is inside the ```/robot``` subfolder, has the following attributes and commands: <br><br>

📦 Command-Based-Code

├── 📂 Commands  `# All robot actions` <br>
│   ├── 📄 Drive Command  `# Teleoperated Command`
│   └── 📄 Auto Drive Command `# Autonomous Command`
├── 📂 Math Functions  `# Utilities/calculations` <br>
│   └── 📄 Calcs (Calculations) <br>
└── 📂 Subsystems  `# Hardware abstractions` <br>
&emsp;&emsp;├── 📄 Drive Subsystem  `# Drivetrain control` <br>
&emsp;&emsp;├── 📄 Constants  `# Configs (motors and joystick's IDs)` <br>
&emsp;&emsp;├── 📄 Main  `# Entry point` <br>
&emsp;&emsp;├── 📄 Robot  `# Main robot class` <br>
&emsp;&emsp;└── 📄 Robot Container  `# Command bindings` 

<br>

---
# 🧠 Code's Logic

The overall logic behind the code hasn't changed much since the Teleoperated version of it, so it works based on the WPILIB library and functions. In the early moments of this code, it has nothing but the teleoperated driving system alone, so, i'll attach to this topic the Teleoperated repositorie README file, which explains the math and the purposes of each functionality in the code.

👉Click [here](https://github.com/raphacnas/Teleop-Code/blob/master/README.md) to get to the README file! 👈

---
## ❓ Where can I find the code?

Unlike the teleoperated's, the command-based code has many .java files working along with each other, and they can be found in the [```src/main/java/frc/robot```](https://github.com/raphacnas/Command-Based-Code/tree/master/src/main/java/frc/robot) folder.
