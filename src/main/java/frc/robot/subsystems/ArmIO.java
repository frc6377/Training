package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface ArmIO {
    public record ArmInput(
        Angle currentAngle,
        AngularVelocity omega
    ) {}

    /**
     * Power out in percent
     * @param power - percentage out (0-1)
     */
    void setPowerOut(double power);
    void periodic();
    ArmInput getInput();
}
