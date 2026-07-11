package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class Autos {
    public static SequentialCommandGroup exampleAuto(SwerveDriveSubsystem swerveDrive) {
        return new SequentialCommandGroup(
            // Example: Drive forward for 2 seconds
            new InstantCommand(() -> swerveDrive.drive(0.5, 0.0, 0.0), swerveDrive),
            new InstantCommand(() -> swerveDrive.drive(0.0, 0.0, 0.0), swerveDrive) // Stop
        );
    }
}