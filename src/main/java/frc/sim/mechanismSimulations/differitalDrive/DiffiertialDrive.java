package frc.sim.mechanismSimulations.differitalDrive;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.sim.mechanismSimulations.Mechanism;

public class DiffiertialDrive implements Mechanism{

    final NetworkTable simulationNT; 
    final StructPublisher<Pose2d> robotPosePublisher;
    
    public static final Time dt = Seconds.of(0.01);

    private final DifferentialDrivetrainSim driveTrainSim;

    private final TalonFX rightDriveLeadMotor;
    private final TalonFXSimState rightDriveLeadSimState;

    private final TalonFX rightDriveFollowMotor;
    private final TalonFXSimState rightDriveFollowSimState;

    private final TalonFX leftDriveLeadMotor;
    private final TalonFXSimState leftDriveLeadSimState;

    private final TalonFX leftDriveFollowMotor;
    private final TalonFXSimState leftDriveFollowSimState;

    private final DifferitalDriveConfig cfg;

    public DiffiertialDrive(DifferitalDriveConfig cfg) {
        this.cfg = cfg;

        rightDriveLeadMotor = new TalonFX(cfg.candIds.rightDriveLead_id());
        rightDriveFollowMotor = new TalonFX(cfg.candIds.rightDriveFollow_id());
        leftDriveLeadMotor = new TalonFX(cfg.candIds.leftDriveLead_id());
        leftDriveFollowMotor = new TalonFX(cfg.candIds.leftDriveFollow_id());

        rightDriveLeadSimState = rightDriveLeadMotor.getSimState();
        rightDriveFollowSimState = rightDriveFollowMotor.getSimState();
        leftDriveLeadSimState = leftDriveLeadMotor.getSimState();
        leftDriveFollowSimState = leftDriveFollowMotor.getSimState();

        LinearSystem<N2, N2, N2> plant = LinearSystemId.createDrivetrainVelocitySystem(
                DCMotor.getKrakenX60(2),
                cfg.physicsConfig.MASS_KG().in(Kilograms),
                cfg.physicsConfig.WHEEL_RADIUS().in(Meters),
                cfg.physicsConfig.TRACK_WIDTH().in(Meters),
                cfg.physicsConfig.ROBOT_MOI(),
                cfg.physicsConfig.GEARING());

        driveTrainSim = new DifferentialDrivetrainSim(
                plant,
                DCMotor.getKrakenX60(2),
                cfg.physicsConfig.GEARING(),
                cfg.physicsConfig.TRACK_WIDTH().in(Meters),
                cfg.physicsConfig.WHEEL_RADIUS().in(Meters),
                null);

        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        simulationNT = instance.getTable("Simulation");
        robotPosePublisher = simulationNT.getStructTopic("Robot Pose", Pose2d.struct).publish();

        driveTrainSim.setState(cfg.physicsConfig.DEFUALT_STATE());
    }

    private final double wheelDistanceToMotorRotations(double trackDistance ){
        return (trackDistance * cfg.physicsConfig.GEARING()) / (2 * Math.PI * cfg.physicsConfig.WHEEL_RADIUS().in(Meters));
    }

    public void setPosition(Pose2d pose){
        driveTrainSim.setState(cfg.physicsConfig.DEFUALT_STATE()); // Remove all velocity
        driveTrainSim.setPose(pose); // goto position and rotation
    }

    @Override
    public void update(Time dt) {
        double leftVoltage = (leftDriveLeadSimState.getMotorVoltage() + leftDriveFollowSimState.getMotorVoltage()) / 2;
        double rightVoltage = (rightDriveLeadSimState.getMotorVoltage() + rightDriveFollowSimState.getMotorVoltage())
                / 2;

        driveTrainSim.setInputs(leftVoltage, rightVoltage);
        driveTrainSim.update(dt.in(Seconds));

        if(DriverStation.isTest()){
            setPosition(new Pose2d(8.27,4.05, new Rotation2d()));
        }

        rightDriveLeadSimState.setRawRotorPosition(wheelDistanceToMotorRotations(driveTrainSim.getRightPositionMeters()));
        leftDriveLeadSimState.setRawRotorPosition(wheelDistanceToMotorRotations(driveTrainSim.getLeftPositionMeters()));
        
        rightDriveLeadSimState.setRotorVelocity(wheelDistanceToMotorRotations(driveTrainSim.getRightVelocityMetersPerSecond()));
        leftDriveLeadSimState.setRotorVelocity(wheelDistanceToMotorRotations(driveTrainSim.getLeftVelocityMetersPerSecond()));
    }

    @Override
    public void updateNTValues() {
        robotPosePublisher.accept(driveTrainSim.getPose());
    }
}
