# TODO: Fix robot movement continuing after switching from autonomous to disabled

## Steps
- [x] 1. Information Gathering - Read all relevant files
- [x] 2. Plan created and approved
- [x] 3. Add `stop()` method to `SwerveDriveSubsystem.java` - resets SlewRateLimiters and calls `drive(0, 0, 0)`
- [x] 4. Add public getter `getSwerveDrive()` to `RobotContainer.java` - exposes swerve drive subsystem
- [x] 5. Add `disabledInit()` to `Robot.java` - calls stop on swerve drive to halt motion
- [x] 6. Build 

