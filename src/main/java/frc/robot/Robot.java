// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.ModuleLayer.Controller;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private final XboxController driver;
  private final TalonFX rightMotorSystem;
  private final TalonFX leftMotorSystem;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
  driver = new XboxController(0);
  rightMotorSystem = new TalonFX(1);
  leftMotorSystem = new TalonFX(3);
  }

  @Override
  public void robotPeriodic() {
    double forward = -driver.getLeftY();
    double right = .775*driver.getRightX();
double rightSpeed = forward + right;
double leftSpeed = forward - right;
rightMotorSystem.set(rightSpeed);
leftMotorSystem.set(leftSpeed);


if (forward == 0) {
    right = right / 10;
}


// when button y is pressed right = 0


    //Controller.
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {


  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
