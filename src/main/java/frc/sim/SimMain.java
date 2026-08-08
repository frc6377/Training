package frc.sim;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

public class SimMain {
    public static final Time dt = Seconds.of(0.01);

    private static final Matrix<N7, N1> DEFUALT_STATE = MatBuilder.fill(N7.instance, N1.instance, 8.27, 4.05, 0, 0, 0, 0, 0);
    private static final Frequency NTupdateFrequency = Hertz.of(60);
    private long nextNTupdate = 0;

    final NetworkTable simulationNT; 
    final StructPublisher<Pose2d> robotPosePublisher;

    private final DifferentialDrivetrainSim driveTrainSim;

    private final TalonFX rightDriveLeadMotor;
    private final TalonFXSimState rightDriveLeadSimState;

    private final TalonFX rightDriveFollowMotor;
    private final TalonFXSimState rightDriveFollowSimState;

    private final TalonFX leftDriveLeadMotor;
    private final TalonFXSimState leftDriveLeadSimState;

    private final TalonFX leftDriveFollowMotor;
    private final TalonFXSimState leftDriveFollowSimState;

    public SimMain() {
        rightDriveLeadMotor = new TalonFX(SimConfig.rightDriveLead_id);
        rightDriveFollowMotor = new TalonFX(SimConfig.rightDriveFollow_id);
        leftDriveLeadMotor = new TalonFX(SimConfig.leftDriveLead_id);
        leftDriveFollowMotor = new TalonFX(SimConfig.leftDriveFollow_id);

        rightDriveLeadSimState = rightDriveLeadMotor.getSimState();
        rightDriveFollowSimState = rightDriveFollowMotor.getSimState();
        leftDriveLeadSimState = leftDriveLeadMotor.getSimState();
        leftDriveFollowSimState = leftDriveFollowMotor.getSimState();

        LinearSystem<N2, N2, N2> plant = LinearSystemId.createDrivetrainVelocitySystem(
                DCMotor.getKrakenX60(2),
                SimConfig.MASS_KG,
                SimConfig.WHEEL_RADIUS.in(Meters),
                SimConfig.TRACK_WIDTH.in(Meters),
                SimConfig.ROBOT_MOI,
                SimConfig.GEARING);

        driveTrainSim = new DifferentialDrivetrainSim(
                plant,
                DCMotor.getKrakenX60(2),
                SimConfig.GEARING,
                SimConfig.TRACK_WIDTH.in(Meters),
                SimConfig.WHEEL_RADIUS.in(Meters),
                null);

        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        simulationNT = instance.getTable("Simulation");
        robotPosePublisher = simulationNT.getStructTopic("Robot Pose", Pose2d.struct).publish();

        setPosition(new Pose2d(8.27,4.05, new Rotation2d()));
    }

    public void periodic() {
        double leftVoltage = (leftDriveLeadSimState.getMotorVoltage() + leftDriveFollowSimState.getMotorVoltage()) / 2;
        double rightVoltage = (rightDriveLeadSimState.getMotorVoltage() + rightDriveFollowSimState.getMotorVoltage())
                / 2;

        driveTrainSim.setInputs(leftVoltage, rightVoltage);
        driveTrainSim.update(dt.in(Seconds));

        if (System.currentTimeMillis() > nextNTupdate) {
            updateNtValues();
        }

        if(DriverStation.isTest()){
            setPosition(new Pose2d(8.27,4.05, new Rotation2d()));
        }
    }

    public void setPosition(Pose2d pose){
        driveTrainSim.setState(DEFUALT_STATE); // Remove all velocity
        driveTrainSim.setPose(pose); // goto position and rotation
    }

    private void updateNtValues() {
        nextNTupdate = (long) (System.currentTimeMillis() + 1 / NTupdateFrequency.in(Hertz));
        robotPosePublisher.accept(driveTrainSim.getPose());
        
    }
}
