package frc.robot;

import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.ironmaple.simulation.SimulatedArena;

import com.pathplanner.lib.auto.AutoBuilder;


public class RobotContainer {
    private final SwerveDriveSubsystem m_swerveDrive = new SwerveDriveSubsystem();
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Configure the default command for the swerve drive
        final CommandXboxController m_driverController = new CommandXboxController(0); // Assuming controller is on port 0
        m_swerveDrive.setDefaultCommand(
            new RunCommand(
                () -> m_swerveDrive.drive(m_driverController.getLeftX(), m_driverController.getLeftY(),  -m_driverController.getRawAxis(2)), // spin in place using right stick X

                m_swerveDrive
            )
        
        );

         // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure button bindings
        configureBindings();
    }

    private void configureBindings() {
        // Add button bindings here (e.g., joystick buttons)
        
    }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

    
}
