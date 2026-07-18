package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private RobotContainer m_robotContainer;
    private Command m_autonomousCommand;

    @Override
    public void robotInit() {
        // Initialize the RobotContainer
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void autonomousInit() {
        // Get the autonomous command from RobotContainer
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousexit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void robotPeriodic() {
        // Run the CommandScheduler for all robot modes
        CommandScheduler.getInstance().run();
    }


    @Override
    public void simulationPeriodic() {
        // Add simulation-specific logic here
    }

  }

