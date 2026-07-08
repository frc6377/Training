// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmIO.ArmInput;

public class ArmSubsystem extends SubsystemBase {
  final ArmIO io;
  ArmInput currentInput;
  ArmConstants constants;
  Mechanism2d mech2d;
  MechanismRoot2d  mechRoot;
  MechanismLigament2d armLigament;


  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem(ArmIO io, ArmConstants constants) {
    this.io = io;
    mech2d = new Mechanism2d(1, 1);
    mechRoot = mech2d.getRoot("arm", 0.5, 0.5);
    armLigament = new MechanismLigament2d("Arm", 0.5, 0);
    mechRoot.append(armLigament);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command gotoPosition(Angle angle) {
    return run(
        () -> {
          double err = angle.minus(currentInput.currentAngle()).in(Degrees);
          double p = err * constants.kP.get();
          double d = 0;
          io.setPowerOut(p+d);
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean atPosition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    io.periodic();
    // This method will be called once per scheduler run
    ArmInput input = io.getInput();
    armLigament.setAngle(input.currentAngle().in(Degrees));
    SmartDashboard.putNumber("arm/angle", input.currentAngle().in(Degrees));

    SmartDashboard.putData("Arm", mech2d);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
