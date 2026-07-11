# TODO

## Step 1
- Gather missing files / compile errors.

## Step 2
- Create simulated swerve subsystem:
  - `SwerveDriveSubsystem` (WPILib kinematics + pose estimator/odometry).
  - `SwerveModule` (holds simulated wheel angle + speed; uses simple first-order response).
  - Minimal constants inside `frc.robot.subsystems` (or reuse `Constants.Swerve`).

## Step 3
- Wire subsystem into `RobotContainer`:
  - Replace placeholder default command with joystick-style inputs (or keep zeros but allow driving via public method).

## Step 4
- Provide a simple autonomous example (optional) that compiles.

## Step 5
- Run `./gradlew build` (or `test`) to confirm compilation.

## Step 6
- If simulation GUI is enabled, ensure drivetrain outputs are published (field visualization/SmartDashboard).

