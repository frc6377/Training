# Training

This is a training resource made by the howdybots.
Each branch is its own project and lesson.

## Getting started

This project includes the [AdvantageKit Skeleton template from v26.0.2](https://github.com/Mechanical-Advantage/AdvantageKit/releases/tag/v26.0.2), using WPILib 2026.2.1 and Phoenix 6. Open this folder in WPILib VS Code with the 2026 toolchain installed.

`Robot.java` sets up AdvantageKit logging and runs the command scheduler. `RobotContainer.java` is empty and ready for subsystem construction and command bindings. The mechanism files in `src/main/java/frc/robot/subsystems/` contain only package, class/interface, and inheritance declarations. Implementing and wiring them is part of the assignment.

## Assignment Overview

Implement three subsystems: an arm, an elevator, and a flywheel. Each must have hardware IO, simulation IO, and AdvantageKit logging.

All motors will use CTRE Talon FX controllers with the Phoenix 6 API.

Simulation is required for all three mechanisms. Complete the hardware IO and simulation IO, then demonstrate the numerical requirements below in desktop simulation with AdvantageScope logs.

### Shared hardware and simulation specifications

| Parameter | Assigned value |
| --- | --- |
| Motors | One Falcon 500 with a Talon FX per mechanism |
| Simulation motor model | `DCMotor.getFalcon500(1)` for each mechanism |
| CAN bus | roboRIO CAN bus (`rio`) |
| Motor CAN IDs | Arm: **1**; elevator: **2**; flywheel: **3** |
| Hardware current limits | Enable a **40 A supply limit** and **60 A stator limit** on each motor |
| Hardware neutral mode | Brake |
| Simulation supply voltage | Fixed **12.0 V** |
| Applied voltage range | **-12.0 V to +12.0 V** |
| Simulation model | Ideal WPILib motor dynamics, zero measurement noise; gravity enabled for arm and elevator |

Gear ratios below mean **motor rotations per output rotation**: a 100:1 reduction requires 100 motor rotations for one mechanism rotation. Positive output must raise the arm, raise the elevator, or increase forward flywheel speed. Make sure to use the same units for targets, measurements, and tolerances.

### Shared IO requirements

Separate subsystem behavior from hardware access. Starter files are provided for each of these types:

| Mechanism | Subsystem | IO interface | Hardware implementation | Simulation implementation |
| --- | --- | --- | --- | --- |
| Arm | `Arm` | `ArmIO` | `ArmIOTalonFX` | `ArmIOSim` |
| Elevator | `Elevator` | `ElevatorIO` | `ElevatorIOTalonFX` | `ElevatorIOSim` |
| Flywheel | `Flywheel` | `FlywheelIO` | `FlywheelIOTalonFX` | `FlywheelIOSim` |

- Pass the IO implementation into the subsystem's constructor.
- Define an `@AutoLog` inputs class and methods for reading inputs and commanding the mechanism in each IO interface.
- Keep Talon FX objects, configuration, sensor reads, and control requests inside the hardware implementation. Commands should interact with the subsystem.
- In the subsystem's periodic loop, call `io.updateInputs(inputs)` and then `Logger.processInputs(...)` before using the inputs. Make control decisions from the logged inputs object.
- Configure motor inversion, neutral mode, current limits, and gearing for the mechanism. Document CAN IDs, the CAN bus, and unit conversions.

Start with [AdvantageKit IO interfaces](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/) for the architecture, [Phoenix 6 configuration](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/configuration.html) for motor setup, and [Phoenix 6 status signals](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html) for reading hardware data.

### Shared logging requirements

- Log measured position where relevant, velocity, applied voltage, motor current, and connection status through the IO inputs. Include any limit-switch or homing inputs used by your code.
- Use `Logger.recordOutput(...)` for the requested goal, the goal after clamping, control error, and whether the mechanism has reached its goal. See [AdvantageKit recording outputs](https://docs.advantagekit.org/data-flow/recording-outputs/).
- Group logs under `Arm`, `Elevator`, and `Flywheel`. Label units clearly, and use matching units for measured values and targets.
- Use an [AdvantageScope line graph](https://docs.advantagescope.org/tab-reference/line-graph/) to compare each mechanism's measured value with its target.

### Shared simulation requirements

- Implement each `IOSim` class using a WPILib physics model with the assigned motor, gearing, dimensions, and mass or inertia below.
- Apply the actual voltage after clamping to the model, advance it once per robot loop using the loop period, and populate the same IO inputs used by the hardware implementation. Simulated measurements must come from the model's state. Apply **0 V** while the robot is disabled.
- Provide a simulation reset that restores the assigned initial position/speed and clears controller and homing state, so the demonstrations can be repeated.
- Demonstrate changes between multiple targets in simulation. Verify arm/elevator travel limits and flywheel stopping, and inspect the logged results in AdvantageScope.

Start with [WPILib physics simulation](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html) for model updates and [the Simulation GUI guide](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/simulation-gui.html) for running and enabling the robot on your computer.

### How the demonstrations are evaluated

Show that each mechanism works in desktop simulation: it reaches its preset targets, holds position or maintains speed, and handles the limit and stop commands described below. Use AdvantageScope logs to make the behavior clear.

Report `atGoal` or `atSpeed` based on the specified tolerance, measured against the goal after clamping. Flywheel `atSpeed` must be false whenever stopped.

### ASSIGNMENT 1. Arm

Build an arm subsystem that moves to a requested angle and holds it against gravity.

| Parameter | Assigned value |
| --- | --- |
| Gear reduction | **100:1**, motor to arm pivot |
| Arm length | **0.60 m**, pivot to tip |
| Arm mass | **4.0 kg**, uniformly distributed along the arm |
| Center of mass | **0.30 m** from the pivot |
| Moment of inertia about the pivot | **0.48 kg·m²** |
| Angle convention | **0°** is horizontal, pointing forward; **90°** is straight up |
| Allowed travel | **-20° to 100°** |
| Initial simulation state | **0°**, **0°/s** |
| Preset angles | **0°**, **45°**, **90°** |

Resources:
- [WPILib: Tuning a vertical arm position controller](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html) explains feedback, feedforward, and gravity compensation with an interactive example.
- [WPILib: ArmFeedforward](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html#armfeedforward) covers feedforward in robot code.
- [Phoenix 6: Closed-loop control](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/closed-loop-requests.html) covers onboard control and arm gravity compensation, including the angle reference and sensor ratios.
- [WPILib: SingleJointedArmSim](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/SingleJointedArmSim.html) documents the arm physics model.

### ASSIGNMENT 2. Elevator

Build an elevator subsystem that moves to a requested height and holds it against gravity.

| Parameter | Assigned value |
| --- | --- |
| Gear reduction | **10:1**, motor to drum |
| Drum radius | **0.025 m** (50 mm diameter) |
| Carriage mass | **5.0 kg** |
| Rigging | Single stage; carriage travel equals cable travel, with no pulley multiplier |
| Allowed travel | **0.00 m to 1.20 m**, measured upward from the bottom |
| Initial simulation state | **0.20 m**, **0.00 m/s**, unhomed |
| Bottom limit switch | **DIO 0**; raw input is false when pressed; simulated switch is pressed at the lower travel limit |
| Homing output | **-1.0 V** |
| Preset heights | **0.20 m**, **0.60 m**, **1.00 m** |

Resources:
- [WPILib: ElevatorFeedforward](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html#elevatorfeedforward) covers feedforward for lifting and holding an elevator.
- [Phoenix 6: Closed-loop control](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/closed-loop-requests.html) covers onboard position control and elevator gravity compensation.
- [WPILib: ElevatorSim](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/ElevatorSim.html) documents the elevator physics model.
- [WPILib: Digital inputs](https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/digital-inputs-software.html) explains reading the bottom limit switch.

### ASSIGNMENT 3. Flywheel

Build a flywheel subsystem that reaches and maintains a requested rotational speed.

| Parameter | Assigned value |
| --- | --- |
| Gear ratio | **1:1**, direct drive |
| Flywheel moment of inertia | **0.004 kg·m²** |
| Initial simulation speed | **0 RPM** |
| Preset speeds | **2,000 RPM**, **3,500 RPM**, **5,000 RPM** |
| Allowed speed goals | **0–5,500 RPM** |
| Stop behavior | Disable velocity control and command **0 V**; report `atSpeed = false` |

Resources:
- [WPILib: Tuning a flywheel velocity controller](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html) explains velocity control and tuning with an interactive example.
- [WPILib: SimpleMotorFeedforward](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html#simplemotorfeedforward) covers feedforward for flywheel control.
- [Phoenix 6: Closed-loop control](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/closed-loop-requests.html) explains onboard velocity control, including voltage-based requests such as `VelocityVoltage`.
- [WPILib: FlywheelSim](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/FlywheelSim.html) documents the flywheel physics model.

### Completion checklist
- [ ] All three subsystems have an IO interface, a Talon FX hardware implementation, and a working simulation implementation.
- [ ] The assigned motor models, gearing, dimensions, IDs, limits, and tolerances are used and stored in the subsystem constants classes.
- [ ] `RobotContainer` constructs the subsystems with the correct IO implementation for each runtime mode and provides command bindings.
- [ ] Commands can change each mechanism's target, and the arm and elevator respect their travel limits.
- [ ] Inputs and calculated outputs are logged with clear names and units.
- [ ] Each mechanism reaches its preset targets within the listed tolerance and holds position or maintains speed.
- [ ] The recorded demonstrations cover out-of-range goals, elevator homing, and flywheel stopping.
- [ ] All three mechanisms have been implemented and tested in desktop simulation, with logs demonstrating arm/elevator limits and holding, plus flywheel spin-up and stopping.
- [ ] You can explain your unit conversions, reference positions, control gains, and how data flows through IO into the logs.



## Reference Material

WPILIB (General Robot Stuff)
- Docs - https://docs.wpilib.org/en/stable/index.html
- Java Docs - [RobotController (WPILib API 2025.3.2)](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/RobotController.html)


Motors
- CTRE Docs - https://v6.docs.ctr-electronics.com/en/stable/
- Rev Robotics Docs - https://docs.revrobotics.com/docs/ion/

Simulation
- WPILIB Simulation Docs - [Introduction to Robot Simulation](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html)
- MapleSim - https://shenzhen-robotics-alliance.github.io/maple-sim/


Advantage Stuff (Data Logging)
- AdvantageKit Docs - https://docs.advantagekit.org/
- AdvantageScope Docs - https://docs.advantagescope.org/


Auton / Vision
- PathPlanner - https://pathplanner.dev/home.html
- AutoPilot - https://therekrab.github.io/autopilot/index.html
- Quest Nav - https://questnav.gg/docs/rewrite
- Quest Nav Java Docs - https://shenzhen-robotics-alliance.github.io/maple-sim/javadocs/
- Lime Light Docs - https://docs.limelightvision.io/docs/docs-limelight/getting-started/summary



Past Howdy Code
- 2026 Code - https://github.com/frc6377/rebuilt_2026/
- 2025 Code - https://github.com/frc6377/reefscape_2025
- 2024 Code - https://github.com/frc6377/crescendo_2024 (Ignore Turret)
- 2023 Code - https://github.com/frc6377/charged_up_2023 (Mostly Irrelevant Now)
