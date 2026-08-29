package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants;


/**
 * Minimal swerve drivetrain intended for simulation.
 *
 * <p>It uses WPILib swerve kinematics to convert commanded chassis speeds into module states.
 * The modules themselves are simulated with simple first-order limits.
 */
public class SwerveDriveSubsystem extends SubsystemBase {

  private final Translation2d m_frontLeftLocation = Constants.Swerve.kFrontLeftLocation;
  private final Translation2d m_frontRightLocation = Constants.Swerve.kFrontRightLocation;
  private final Translation2d m_backLeftLocation = Constants.Swerve.kBackLeftLocation;
  private final Translation2d m_backRightLocation = Constants.Swerve.kBackRightLocation;

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveModule m_frontLeft = new SwerveModule();
  private final SwerveModule m_frontRight = new SwerveModule();
  private final SwerveModule m_backLeft = new SwerveModule();
  private final SwerveModule m_backRight = new SwerveModule();

  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3.0);

  private final Field2d m_field = new Field2d();

  private final double m_maxSpeedMetersPerSecond = 4.0;

  private Pose2d m_pose = new Pose2d();
  private Rotation2d m_heading = new Rotation2d();

  private double m_lastTimestamp = Timer.getFPGATimestamp();

  public SwerveDriveSubsystem() {
    SmartDashboard.putData("Field", m_field);
    RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

      AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  /**
   * Drive the robot.
   *
   * @param xSpeedMetersPerSecond forward (+) m/s (robot-relative)
   * @param ySpeedMetersPerSecond left (+) m/s (robot-relative)
   * @param rotRadiansPerSecond CCW (+) rad/s
   */
  /**
   * Immediately stop all drivetrain motion.
   * Resets slew rate limiters so subsequent drive commands start from zero.
   */
  public void stop() {
    m_xLimiter.reset(0.0);
    m_yLimiter.reset(0.0);
    m_rotLimiter.reset(0.0);
    drive(0, 0, 0);
  }

  public void drive(double xSpeedMetersPerSecond, double ySpeedMetersPerSecond, double rotRadiansPerSecond) {
    // Clamp + slew for nicer sim.
    double vx = m_xLimiter.calculate(xSpeedMetersPerSecond);
    double vy = m_yLimiter.calculate(ySpeedMetersPerSecond);
    double omega = m_rotLimiter.calculate(rotRadiansPerSecond);

    vx = clamp(vx, -m_maxSpeedMetersPerSecond, m_maxSpeedMetersPerSecond);
    vy = clamp(vy, -m_maxSpeedMetersPerSecond, m_maxSpeedMetersPerSecond);
    omega = clamp(omega, -Math.PI, Math.PI);

    ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);

    // Convert to module states (m/s wheel speed).
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    // Normalize wheel speeds so none exceed max.
    double realMax = 0.0;
    for (SwerveModuleState s : moduleStates) {
      realMax = Math.max(realMax, Math.abs(s.speedMetersPerSecond));
    }
    if (realMax > m_maxSpeedMetersPerSecond) {
      double scale = m_maxSpeedMetersPerSecond / realMax;
      for (int i = 0; i < moduleStates.length; i++) {
        moduleStates[i] = new SwerveModuleState(moduleStates[i].speedMetersPerSecond * scale, moduleStates[i].angle);
      }
    }

    // Send desired state to modules.
    m_frontLeft.setDesiredState(
        new SwerveModule.SwerveModuleState(
            moduleStates[0].angle, moduleStates[0].speedMetersPerSecond));
    m_frontRight.setDesiredState(
        new SwerveModule.SwerveModuleState(
            moduleStates[1].angle, moduleStates[1].speedMetersPerSecond));
    m_backLeft.setDesiredState(
        new SwerveModule.SwerveModuleState(
            moduleStates[2].angle, moduleStates[2].speedMetersPerSecond));
    m_backRight.setDesiredState(
        new SwerveModule.SwerveModuleState(
            moduleStates[3].angle, moduleStates[3].speedMetersPerSecond));
        
  
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    double dt = now - m_lastTimestamp;
    m_lastTimestamp = now;

    dt = Math.max(0.001, Math.min(0.05, dt));

    // Update simulated modules.
    m_frontLeft.update(dt);
    m_frontRight.update(dt);
    m_backLeft.update(dt);
    m_backRight.update(dt);

    // Estimate chassis motion by integrating commanded chassis speeds via odometry.
    // For this sim, we treat the robot as moving according to the kinematics-reconstructed
    // chassis speeds from the *current* module states.
    SwerveModulePosition[] positions = new SwerveModulePosition[] {
      new SwerveModulePosition(m_frontLeft.getSpeedMetersPerSecond() * dt, m_frontLeft.getAngle()),
      new SwerveModulePosition(m_frontRight.getSpeedMetersPerSecond() * dt, m_frontRight.getAngle()),
      new SwerveModulePosition(m_backLeft.getSpeedMetersPerSecond() * dt, m_backLeft.getAngle()),
      new SwerveModulePosition(m_backRight.getSpeedMetersPerSecond() * dt, m_backRight.getAngle())
    };

    // Reconstruct chassis speeds by inverting kinematics from module states.
    // We'll use the forward integration style: compute chassis twist from module velocities.
    // Since we only have distances per dt, approximate twist using those.
    // (Good enough for a simple sim.)
    //
    // WPILib doesn't provide direct inverse of module distances->twist, so we use the
    // commanded module states as the closest proxy by simply tracking heading and x/y
    // using the current heading and wheel speed averages.

    // Update heading from module rotational intent.
    // We approximate omega from heading change = current desired omega is not stored,
    // so instead we use an average wheel velocity projection around center is complex.
    // For sim purposes: keep heading integration using module angles+speeds.

    // Simpler: use the kinematics to estimate chassis speeds from current module states.
    SwerveModuleState[] states = new SwerveModuleState[] {
      new SwerveModuleState(m_frontLeft.getSpeedMetersPerSecond(), m_frontLeft.getAngle()),
      new SwerveModuleState(m_frontRight.getSpeedMetersPerSecond(), m_frontRight.getAngle()),
      new SwerveModuleState(m_backLeft.getSpeedMetersPerSecond(), m_backLeft.getAngle()),
      new SwerveModuleState(m_backRight.getSpeedMetersPerSecond(), m_backRight.getAngle())
    };


    ChassisSpeeds chassisFieldRelative = null;
    try {
      // toChassisSpeeds assumes robot-relative.
      chassisFieldRelative = m_kinematics.toChassisSpeeds(states);
    } catch (Exception e) {
      // Should not happen, but keep sim stable.
      chassisFieldRelative = new ChassisSpeeds(0, 0, 0);
    }

    // chassisFieldRelative currently is robot-relative.
    double vxRobot = chassisFieldRelative.vxMetersPerSecond;
    double vyRobot = chassisFieldRelative.vyMetersPerSecond;
    double omega = chassisFieldRelative.omegaRadiansPerSecond;

    // Integrate pose.
    m_heading = m_heading.plus(new Rotation2d(omega * dt));
    Translation2d fieldVel = new Translation2d(
        vxRobot * m_heading.getCos() - vyRobot * m_heading.getSin(),
        vxRobot * m_heading.getSin() + vyRobot * m_heading.getCos());

    m_pose = new Pose2d(
        m_pose.getX() + fieldVel.getX() * dt,
        m_pose.getY() + fieldVel.getY() * dt,
        m_heading);

    m_field.setRobotPose(m_pose);

    SmartDashboard.putNumber("Swerve/X", m_pose.getX());
    SmartDashboard.putNumber("Swerve/Y", m_pose.getY());
    SmartDashboard.putNumber("Swerve/HeadingDeg", m_heading.getDegrees());
    SmartDashboard.putData("Field", m_field);
  }

  public Pose2d getPose() {
    return m_pose;
  }
  public void resetPose(Pose2d pose) {
    m_pose = pose;
    m_heading = pose.getRotation();
  }
  public ChassisSpeeds getRobotRelativeSpeeds() {
    // Return the current robot-relative speeds based on module states.
    SwerveModuleState[] states = new SwerveModuleState[] {
      new SwerveModuleState(m_frontLeft.getSpeedMetersPerSecond(), m_frontLeft.getAngle()),
      new SwerveModuleState(m_frontRight.getSpeedMetersPerSecond(), m_frontRight.getAngle()),
      new SwerveModuleState(m_backLeft.getSpeedMetersPerSecond(), m_backLeft.getAngle()),
      new SwerveModuleState(m_backRight.getSpeedMetersPerSecond(), m_backRight.getAngle())
    };

    return m_kinematics.toChassisSpeeds(states);
  }
  private static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }
}

