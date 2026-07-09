package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ArmSimIO implements ArmIO {
    SingleJointedArmSim armSim;
    ArmConstants defaultArm = new ArmConstants(1, Kilogram.of(5), 1);
    EventLoop loop;
    boolean grav = false;

    public ArmSimIO(){
        armSim = createArm(defaultArm, false);
        loop = new EventLoop();
        SmartDashboard.putBoolean("arm/sim/gravity", false);
    }

    public void setGravity(boolean gravity){
        SingleJointedArmSim newArm = createArm(defaultArm, gravity);
        newArm.setState(armSim.getOutput());
        armSim = newArm;
    }

    public static SingleJointedArmSim createArm(ArmConstants constants, boolean gravity){
        SingleJointedArmSim arm = 
            new SingleJointedArmSim(
                DCMotor.getKrakenX60(1), 
                constants.gearing(), 
                constants.moi(), 
                constants.length(), 
                -3140, 
                3140, 
                gravity, 
                0);
        return arm;
    }

    @Override
    public void setPowerOut(double power) {
        armSim.setInputVoltage(12 * MathUtil.clamp(power, -1, 1));
    }

    @Override
    public void periodic() {
        armSim.update(0.020);
        
        boolean newGrav = SmartDashboard.getBoolean("arm/sim/gravity", false);
        if(newGrav != grav){
            setGravity(newGrav);
        }
        grav = newGrav;
    }

    @Override
    public ArmInput getInput() {
        // return new ArmInput(Radians.of(1), RadiansPerSecond.of(0));
        return new ArmInput(Radians.of(armSim.getAngleRads()), RadiansPerSecond.of(armSim.getVelocityRadPerSec()));
    }
    
    record ArmConstants(double gearing, Mass weightKg, double length){
        double moi(){
            return SingleJointedArmSim.estimateMOI(length, weightKg.in(Kilogram));
        }
    }
}
