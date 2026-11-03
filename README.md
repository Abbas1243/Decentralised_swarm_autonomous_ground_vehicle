# BE CAPSTONE PROJECT 

## 🤖 Decentralized Swarm Autonomous Ground Vehicles
<p align="left">
  <a href="https://youtu.be/RQamKgik0cE?si=gTgPMuQe8ZkZP1P7">
    <img src="https://img.shields.io/badge/Project-Video%20Link-blue?style=for-the-badge" alt="Project Video">
  </a>
</p>



---
## 👥 Team Details

| **Sr. No.** | **Name of Student** | **Roll No.** | **Branch** | **Email ID** |
|:---:|:---|:---:|:---|:---|
| 1 | Abbas Najam | 59 |Automation and Robotics | [2023.abbas.najam@ves.ac.in](mailto:2023.abbas.najam@ves.ac.in) |
| 2 | Mayuresh Joshi | 49 | Automation and Robotics | [2023.mayuresh.joshi@ves.ac.in](mailto:2023.mayuresh.joshi@ves.ac.in) |
| 3 | Shravan Umrale | 66 | Automation and Robotics | [2023.shravan.umrale@ves.ac.in](mailto:2023.shravan.umrale@ves.ac.in) |
| 4 | Yash Vaychal | 31 | Automation and Robotics | [2023.yash.vaychal@ves.ac.in](mailto:2023.yash.vaychal@ves.ac.in) |

---

## Guide Details

**Project Guide:**  Deepti Khimani

**Department:** Automation and Robotics  

**Institute:** VESIT, Mumbai

# 📌 Project Overview

Swarm robotics is inspired by the collective behavior observed in nature, such as ants, bees and birds. Many simple units working together can cover more ground, and tolerate the loss of individual units better, than a single capable unit.

This project focuses on the design and development of a **six-robot low-cost differential-drive robotic swarm for cooperative indoor exploration and mapping**.

Each robot is designed around an ESP32-WROOM-32E microcontroller mounted on a custom-designed PCB. The PCB integrates the motor driver and DC-DC buck converter, reducing the need for separate electronic modules and simplifying the overall hardware assembly.

Each robot is equipped with:
- Six VL53L0X Time-of-Flight (ToF) distance sensors.
- One MPU6050 six-axis IMU.
- Two N20 geared DC motors with integrated wheel encoders.
- Two 34 mm rubber drive wheels.
- One ball caster.
- One 7.4 V, 2600 mAh LiPo battery.

The mechanical structure consists of a 3 mm laser-cut MDF base plate, custom 3D-printed PLA motor holders, a caster holder and spacers. M3 nuts and bolts are used for mechanical assembly.

The current implementation assumes that the robots begin from predefined and known starting positions and orientations. Estimating unknown starting poses and automatically aligning independently initialized robot maps are considered future extensions.


### Key Capabilities
Key Capabilities
* 🤖 Low-cost differential-drive robots (ESP32 on custom PCB)
* 📏 6-beam ToF sensing per robot
* 🧭 Odometry from wheel encoders fused with IMU gyro
* 🗺️ Per-robot SLAM using pose graph and scan matching
* 🧩 Merging of all robots' observations into one map
* 📡 Wi-Fi telemetry to a laptop and robot-to-robot pose sharing
* 🚧 Reactive obstacle avoidance and wall following
* 🧠 Decentralized exploration behaviour
* 📊 Live map viewer and replayable data logs
* 📐 Evaluation against tape-measured ground truth
---

# 🎯 Problem Statement

Most swarm robotic systems currently exist in a **centralized architecture**, where a central controller is responsible for coordinating the robots.

However, in real-world applications, we cannot rely on a single point of failure. If the central controller fails or communication with it is lost, the entire swarm can be affected.

Therefore, this project aims to develop a **decentralized swarm system with fault detection and recovery mechanisms**, where individual robots can communicate with neighboring robots, make local decisions, detect failures, and continue the mission collectively.

---

# 🚀 Objectives

The major objectives of this project are:

* Develop a decentralized multi-robot swarm system.
* Enable communication between multiple autonomous ground vehicles.
* Implement autonomous navigation.
* Implement mapping and localization.
* Develop obstacle avoidance capabilities.
* Implement distributed decision-making.
* Introduce swarm intelligence algorithms.
* Detect communication, sensor, and motor-related faults.
* Develop fault recovery mechanisms.
* Reallocate tasks when a robot becomes unavailable.
* Maintain swarm operation after individual robot failure.
* Validate the system through simulation.
* Develop and test physical AGV prototypes.
* Evaluate swarm performance under different fault conditions.

---
## Existing System

Describe the currently available system or method.

Mention its limitations:

- High cost
- Low accuracy
- Manual process
- Lack of automation
- Poor scalability
- Limited accessibility

---

## Proposed System

Describe your proposed solution.

Include:

- Main idea
- How it works
- Major components
- Expected benefits

---

## System Architecture

Add block diagram or system architecture image here.

```markdown
![System Architecture](images/system_architecture.png)
````

Briefly explain the architecture.

---


## Hardware Requirements

| Sr. No. | Component | Specification / Model | Quantity |
|:---:|---|---|:---:|
| 1 | Microcontroller | ESP32-WROOM-32E | 6 |
| 2 | Custom PCB | ESP32, motor driver and buck converter integration | 6 |
| 3 | Motor Driver | Integrated on custom PCB | 6 |
| 4 | DC-DC Buck Converter | Integrated on custom PCB | 6 |
| 5 | IMU | MPU6050 (6-axis) | 6 |
| 6 | ToF Sensors | VL53L0X, 6 sensors per robot | 36 |
| 7 | N20 Motors | Geared DC motors with integrated encoders | 12 |
| 8 | Wheel Encoders | Integrated with N20 motors | 12 |
| 9 | Battery | LiPo 7.4 V, 2600 mAh (2S) | 6 |


##  Mechanical Components

| Sr. No. | Component | Specification | Quantity |
|:---:|---|---|:---:|
| 1 | Base Plate | 3 mm MDF, laser cut | 6 |
| 2 | Drive Wheels | 34 mm Mini Car N20 Motor Wheel, rubber | 12 |
| 3 | Caster Wheel | Mini 3Pi Car N20 Caster Robot Ball Wheel | 6 |
| 4 | Motor Holders | PLA 3D printed | 6 sets |
| 5 | Caster Holders | PLA 3D printed | 6 sets |
| 6 | Spacers | PLA 3D printed | 6 sets |
| 7 | Fasteners | M3 nut-and-bolt assembly | As required |


##  Mechanical and Sensor Component Specifications

| Sr. No. | Component | Model / Specification | Quantity | Product Link |
|:---:|---|---|:---:|---|
| 1 | N20 DC Gear Motor | 3V, 150 RPM, with integrated encoder | 12 | [Robu.in](https://robu.in/product/n20-3v-150rpm-micro-metal-gear-motor-with-encoder/) |
| 2 | ToF Distance Sensor | VL53L0X, I2C interface | 36 | [Robu.in](https://robu.in/product/vl53l0x-tof-based-lidar-laser-distance-sensor/) |
| 3 | Drive Wheel | 34 mm Mini Car N20 Motor Wheel, Rubber | 12 | [Robu.in](https://robu.in/product/34mm-mini-car-n20-motor-wheel-rubber-small-wheel/) |
| 4 | Caster Wheel | Mini 3Pi Car N20 Caster Robot Ball Wheel | 6 | [Robu.in](https://robu.in/product/mini-3pi-car-n20-caster-robot-ball-wheel/) |
---

## Software Requirements

| Sr. No. | Software / Tool | Version | Purpose |
| ------- | --------------- | ------- | ------- |
| 1       |                 |         |         |
| 2       |                 |         |         |
| 3       |                 |         |         |

---

## Technologies Used

Mention technologies used in the project.

Example:

* Embedded C / Python / JavaScript
* Arduino / STM32 / ESP32 / Raspberry Pi
* ROS / MATLAB / Simulink
* Machine Learning / Computer Vision
* IoT / Cloud / Mobile App
* PCB Design / CAD Design

---

## Methodology

Explain the step-by-step approach.

1. Literature survey
2. Problem identification
3. Requirement analysis
4. System design
5. Hardware/software development
6. Integration
7. Testing and validation
8. Documentation and publication

---

## Project Timeline

| Week / Month | Task Planned          | Status                            |
| ------------ | --------------------- | --------------------------------- |
| Week 1       | Problem finalization  | Pending / In Progress / Completed |
| Week 2       | Literature survey     |                                   |
| Week 3       | Requirement analysis  |                                   |
| Week 4       | System design         |                                   |
| Week 5       | Prototype development |                                   |
| Week 6       | Testing               |                                   |
| Week 7       | Documentation         |                                   |
| Week 8       | Paper writing         |                                   |

---

## Weekly Progress Updates

Students must update this section every week.

| Week   | Date | Work Completed | Work Planned for Next Week | Issues / Challenges | GitHub Commit Link |
| ------ | ---- | -------------- | -------------------------- | ------------------- | ------------------ |
| Week 1 |      |                |                            |                     |                    |
| Week 2 |      |                |                            |                     |                    |
| Week 3 |      |                |                            |                     |                    |
| Week 4 |      |                |                            |                     |                    |
| Week 5 |      |                |                            |                     |                    |
| Week 6 |      |                |                            |                     |                    |
| Week 7 |      |                |                            |                     |                    |
| Week 8 |      |                |                            |                     |                    |

---

## Design Files

Upload and link all design files here.

| File Type       | File Name / Link | Description |
| --------------- | ---------------- | ----------- |
| CAD Model       |                  |             |
| Circuit Diagram |                  |             |
| PCB Design      |                  |             |
| Flowchart       |                  |             |
| Simulation File |                  |             |

---

## Circuit Diagram

Add circuit diagram image here.

```markdown
![Circuit Diagram](images/circuit_diagram.png)
```

---

## Flowchart / Algorithm

Add flowchart image here.

```markdown
![Flowchart](images/flowchart.png)
```

### Algorithm

1. Start
2. Initialize the system
3. Read input from sensors/user
4. Process the data
5. Generate output/control action
6. Display/store/transmit result
7. Stop

---

## Implementation Details

Explain the actual implementation of the project.

### Hardware Implementation

Write details about connections, components, power supply, sensors, actuators, PCB, enclosure, etc.

### Software Implementation

Write details about code structure, libraries used, algorithms, communication protocols, database, app, cloud, etc.

---

## Code Structure

```text
BE-Capstone-Project/
│
├── README.md
├── docs/
│   ├── literature_survey.md
│   ├── project_report.pdf
│   └── presentation.pptx
│
├── hardware/
│   ├── circuit_diagram.png
│   ├── pcb_design/
│   └── cad_model/
│
├── software/
│   ├── src/
│   ├── include/
│   └── tests/
│
├── images/
│   ├── system_architecture.png
│   ├── prototype_photo.jpg
│   └── results.png
│
└── references/
    └── papers/
```

---

## How to Run the Project

### Step 1: Clone the Repository

```bash
git clone https://github.com/username/project-name.git
```

### Step 2: Install Dependencies

```bash
pip install -r requirements.txt
```

or mention specific software/library installation steps.

### Step 3: Upload / Run the Code

```bash
python main.py
```

or

```bash
arduino-cli upload -p COMx --fqbn board_name
```

### Step 4: Observe the Output

Mention the expected output of the project.

---

## Testing and Results

| Test No. | Test Description | Expected Result | Actual Result | Status      |
| -------- | ---------------- | --------------- | ------------- | ----------- |
| 1        |                  |                 |               | Pass / Fail |
| 2        |                  |                 |               | Pass / Fail |
| 3        |                  |                 |               | Pass / Fail |

---

## Result Images / Videos

Add images or videos of the working prototype.

```markdown
![Prototype](images/prototype_photo.jpg)
```

Video Link: https://youtu.be/-i1T_cCoHbY

```markdown
[Project Demo Video](https://drive.google.com/your-video-link)
```

---

## Applications

1. Education and Research
   - A cheap platform for students and labs to study multi-robot SLAM and swarm behaviour without expensive sensors.

2. Indoor Mapping of Unknown Spaces
   - Multiple low-cost robots can map rooms or corridors faster than one robot.

3. Search and Rescue (Prototype Level)
   - Many inexpensive, expendable robots can explore collapsed or hazardous indoor areas.

4. Industrial and Warehouse Inspection
   - Small robots can map aisles and detect layout changes, cheaply and in parallel.

5. Hazardous Environment Exploration
   - Low-cost units can be risked in places that are unsafe for people.

6. Platform for Larger Swarm Experiments
   - The design can scale to more robots because each unit is inexpensive.

---

## Advantages


1. **Low-Cost Hardware Architecture**

   Uses ESP32 microcontrollers and integrated custom PCBs instead of expensive onboard computing platforms.

2. **Six-Robot Experimental Platform**

   Allows investigation of multi-robot cooperation rather than limiting experimentation to a single robotic platform.

3. **Custom PCB Integration**

   Integrates the microcontroller, motor driver and buck converter on a common PCB, reducing separate modules and wiring.

4. **Modular Mechanical Design**

   Laser-cut MDF and 3D-printed PLA components make mechanical modifications and replacement easier.

5. **ROS 2 Integration**

   Provides a flexible software environment for developing robot communication, mapping and visualization.

6. **Scalable Experimental Architecture**

   Provides a foundation for studying the addition of further robots, subject to communication, processing and hardware limitations.

7. **Practical Engineering Experience**

   Involves PCB development, embedded programming, mechanical design, 3D printing, ROS 2 and multi-robot system integration.

---

## Limitations

1. Sparse and Short-Range Sensing
   - VL53L0X sensors are single-zone, with about a 25 degree cone and a practical indoor range of about 1.2 m, worse on dark, glossy or sunlit surfaces. Scans are sparse compared with lidar.

2. Centralized Mapping
   - Mapping and optimization run on the laptop, so the system is not fully decentralized and depends on the laptop and Wi-Fi link for the merged map.

3. Accuracy Gain Only Where Paths Overlap
   - Away from overlapping regions each robot drifts like a single robot. Wheel slip and heading drift dominate odometry error.

4. Known Starting Poses
   - The current scope assumes known starting poses. Merging with unknown starting poses is future work.

5. Environment Sensitivity
   - Symmetric rooms and long featureless corridors make scan matching less reliable.

6. Battery and Communication Limits
   - Limited battery life, and Wi-Fi channel sharing between the laptop link and robot-to-robot messages can cause packet loss.

---

## Future Scope 

1. Unknown Starting Poses
   - Align robots' maps without known starts using multi-resolution correlative matching of submaps, with strict acceptance gates (score, overlap, margin, repeated consistency) to prevent wrong merges, and a measured false-merge rate in symmetric arenas.

2. Inter-Robot Relative Observations
   - Use rendezvous or markers to add direct robot-to-robot constraints to the pose graph.

3. Sensor Upgrade
   - Move to multizone ToF sensors (for example VL53L5CX) for denser scans if the VL53L0X array proves too sparse. Cost, size and driver complexity must be checked first.

4. More Decentralized Mapping
   - Move parts of mapping onto the robots, and elect a robot to run optimization, as in the ETH swarm work.

5. Fault Detection and Task Reallocation
   - Carry over ideas from the earlier project: detect communication, sensor and motor faults, and reassign exploration to other robots.

6. Ground-Truth Evaluation
   - Overhead camera with printed markers on the robots for pose error over time, used for evaluation only.


---

## Research Paper / Publication

| Item                      | Details                                                   |
| ------------------------- | --------------------------------------------------------- |
| Paper Title               |                                                           |
| Conference / Journal Name |                                                           |
| Paper Status              | Not Started / Drafting / Submitted / Accepted / Published |
| Submission Date           |                                                           |
| Paper Link                |                                                           |

---

## References

Add references in IEEE format.

Example:

```text
[1] A. Author, B. Author, "Title of the Paper," Journal/Conference Name, vol. X, no. Y, pp. xx-yy, Year.
[2] Datasheet / Website / Book reference.
```

---

## Repository Update Guidelines

Each student team must update the GitHub repository regularly.

Minimum expected updates:

* Update README every week.
* Push code changes regularly.
* Upload circuit diagrams, CAD files, PCB files, reports and presentations.
* Add weekly progress in the progress table.
* Maintain proper folder structure.
* Do not upload unnecessary temporary files.
* Each major update should have a meaningful commit message.

Example commit messages:

```text
Added problem statement and objectives
Updated system architecture diagram
Added sensor interfacing code
Updated weekly progress for Week 3
Added testing results and prototype images
```

---

## Declaration

We declare that this project work is carried out by our team as part of the BE Capstone Project. The work will be regularly updated on GitHub and all references used will be properly cited.

---

## License

This project is for academic use only.

Optional:

```text
MIT License / Creative Commons / Institute Use Only
```

```
```
