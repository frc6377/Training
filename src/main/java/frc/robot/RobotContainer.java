package frc.robot;

import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    private final SwerveDriveSubsystem m_swerveDrive = new SwerveDriveSubsystem();

    public RobotContainer() {
        // Configure the default command for the swerve drive
        final CommandXboxController m_driverController = new CommandXboxController(0); // Assuming controller is on port 0
        m_swerveDrive.setDefaultCommand(
            new RunCommand(
                () -> m_swerveDrive.drive(-m_driverController.getLeftX(), -m_driverController.getLeftY(), -m_driverController.getRightX()), // Replace with joystick inputs
                m_swerveDrive
            )
        );

        // Configure button bindings
        configureBindings();
    }

    private void configureBindings() {
        // Add button bindings here (e.g., joystick buttons)
        
    }

    public Command getAutonomousCommand() {
        // Return the autonomous command
        return null; // Replace with an actual autonomous command if needed
    }
}