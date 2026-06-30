# Parameterized MRL Traction Lift Controller (Verilog HDL)

This repository implements a parameterized, digital control system for a Machine Room-Less (MRL) traction elevator using Verilog HDL. Designed for FPGA target implementation (such as the Xilinx Basys 3 board), the controller coordinates lift movement, door cycles, safety loops, and scheduling using the LOOK algorithm.

---

## 1. Physical Elevator Concept (Beginner-Friendly)

An elevator is a complex electromechanical system. This controller is designed for a **Machine Room-Less (MRL) Traction Elevator**, which operates on the following principles:

### A. Machine Room-Less (MRL) Layout
Traditional elevators require a separate machine room above the elevator shaft to house the heavy motor and control cabinets. MRL elevators save building space by:
* Mounting a compact, gearless Permanent Magnet Synchronous Motor (PMSM) directly to the guide rails inside the elevator shaft.
* Placing the controller electronics cabinet inside the shaft wall at the top landing.

### B. Counterweight Balancing
The elevator cabin is connected to a heavy counterweight by steel hoisting cables wrapped around the motor sheave (pulley).
* The counterweight is filled to match the weight of the **empty cabin + 50% of the maximum weight capacity**.
* This balances the gravitational forces, meaning the motor only needs to lift the net difference in weight, vastly reducing energy consumption.

### C. Sensors (Inputs to the Controller)
* **Floor Position Sensors (`floor_sensor`)**: Proximity switches mounted at each floor. When the cabin passes a floor, the switch closes, generating a signal.
* **Door Obstruction Detector (`door_obstruction`)**: A light curtain/photo-eye on the cabin doors that detects if an object is blocking the doors.
* **Overload Sensor (`cabin_overload`)**: A load cell under the cabin floor that detects if the passenger weight limit is exceeded.
* **Buttons (`hall_req_up`, `hall_req_dn`, `cabin_req`)**: Hall buttons on the floor landings and destination buttons inside the cabin.

### D. Actuators (Outputs from the Controller)
* **Traction Motor (`motor_up` / `motor_dn`)**: Drive lines to rotate the sheave forward or backward.
* **Mechanical Brake (`brake_release`)**: An electromagnetic coil that holds the mechanical brake shut when de-energized. Energizing it releases the brake, allowing the motor to rotate.
* **Door Motor (`door_open` / `door_close`)**: Drives the cabin door operator to open or close the door set.

---

## 2. Digital Controller Logic

The digital controller is implemented in [lift_controller.v](file:///home/tyler/workspace/lift_controller/lift_controller.srcs/sources_1/new/lift_controller.v). It operates in two main layers:

### A. The LOOK Scheduling Algorithm
Instead of moving to the closest button press (which could cause the lift to oscillate back and forth and starve passengers at extreme floors), the controller uses the **LOOK (SCAN) algorithm**:
1. The lift continues moving in its current direction (`UP` or `DOWN`) as long as there are pending requests in that direction.
2. It services floors sequentially along its path.
3. It only reverses direction when there are no more requests remaining in the current direction.

### B. Finite State Machine (FSM)
The controller uses a 3-bit Gray-code encoded Moore FSM to coordinate system states:
* **`IDLE` (`3'b000`)**: Cabin is stationary with doors closed.
* **`MOVE_UP` (`3'b001`)**: Cabin moves upwards to the target.
* **`MOVE_DOWN` (`3'b011`)**: Cabin moves downwards to the target.
* **`DOOR_OPEN` (`3'b010`)**: Opens doors and initializes the door dwell countdown.
* **`DOOR_HOLD` (`3'b110`)**: Keeps doors open for 3 seconds, extending if a door obstruction or overload is detected.

---

## 3. Verilog Module Interface

The port map for [lift_controller.v](file:///home/tyler/workspace/lift_controller/lift_controller.srcs/sources_1/new/lift_controller.v) is detailed below:

| Port Name | Direction | Width | Description |
| :--- | :--- | :--- | :--- |
| **`clk`** | Input | 1 bit | System clock (50 MHz) |
| **`rst`** | Input | 1 bit | Synchronous active-high reset |
| **`hall_req_up`** | Input | `NUM_FLOORS` | Hall call up buttons (one per floor) |
| **`hall_req_dn`** | Input | `NUM_FLOORS` | Hall call down buttons (one per floor) |
| **`cabin_req`** | Input | `NUM_FLOORS` | Cabin destination buttons (one per floor) |
| **`floor_sensor`** | Input | `NUM_FLOORS` | One-hot position indicators (1 = cabin at floor N) |
| **`door_obstruction`** | Input | 1 bit | Obstacle detection switch (1 = obstructed) |
| **`cabin_overload`**| Input | 1 bit | Load limit warning switch (1 = overloaded) |
| **`motor_up`** | Output | 1 bit | Drive motor UP command |
| **`motor_dn`** | Output | 1 bit | Drive motor DOWN command |
| **`brake_release`** | Output | 1 bit | Release mechanical brake |
| **`door_open`** | Output | 1 bit | Actuate door open |
| **`door_close`** | Output | 1 bit | Actuate door close |

---

## 4. How to Open and Simulate the Project

This project contains a pre-configured Xilinx Vivado workspace setup.

### Prerequisites
* Xilinx Vivado Design Suite (2020.1 or newer recommended)

### Opening the Project
1. Launch Vivado.
2. Click **Open Project**.
3. Browse to `/home/tyler/workspace/lift_controller/` and select `lift_controller.xpr`.

### Running Behavioral Simulation
1. In the **Sources** pane, verify [lift_controller_tb.v](file:///home/tyler/workspace/lift_controller/lift_controller.srcs/sim_1/new/lift_controller_tb.v) is set as the active simulation top module.
2. In the left-hand Flow Navigator, click **Run Simulation** -> **Run Behavioral Simulation**.
3. Use the wave viewer to analyze FSM state transitions, timer registers, and motor control output waveforms.

---

## 5. Repository Structure

* [lift_controller.v](file:///home/tyler/workspace/lift_controller/lift_controller.srcs/sources_1/new/lift_controller.v): Main RTL module.
* [lift_controller_tb.v](file:///home/tyler/workspace/lift_controller/lift_controller.srcs/sim_1/new/lift_controller_tb.v): System testbench simulating landing requests, passenger boarding, obstructions, and overload events.
* [references/](file:///home/tyler/workspace/lift_controller/references/): Project design drawings, mechanical blueprints, and FSM transition charts.
  * [lift_hardware_design.md](file:///home/tyler/workspace/lift_controller/references/lift_hardware_design.md): Labeled blueprints of the shaft layout and controller cabinet.
  * [lift_controller_fsm.md](file:///home/tyler/workspace/lift_controller/references/lift_controller_fsm.md): Mermaid.js state transition diagrams.
