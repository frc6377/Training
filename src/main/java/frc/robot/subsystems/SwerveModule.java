package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Simple simulated swerve module.
 *
 * <p>This is not tied to any motor controllers—it's intended purely for desktop simulation and
 * basic testing of drivetrain kinematics.
 */
public class SwerveModule {
  private Rotation2d m_currentAngle = new Rotation2d();
  private double m_currentSpeedMetersPerSecond = 0.0;

  // Very simple dynamics limits for nicer simulation.
  private final double m_maxAngleRadPerSecond;
  private final double m_maxSpeedMetersPerSecondPerSecond;

  private Rotation2d m_targetAngle = new Rotation2d();
  private double m_targetSpeedMetersPerSecond = 0.0;

  public SwerveModule() {
    this(10.0, 4.0);
  }

  public SwerveModule(double maxAngleRadPerSecond, double maxSpeedMetersPerSecondPerSecond) {
    m_maxAngleRadPerSecond = maxAngleRadPerSecond;
    m_maxSpeedMetersPerSecondPerSecond = maxSpeedMetersPerSecondPerSecond;
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    m_targetAngle = desiredState.angle;
    m_targetSpeedMetersPerSecond = desiredState.speedMetersPerSecond;
  }

  /**
   * Advance module internal state by dt.
   *
   * @param dtSeconds delta time in seconds
   */
  public void update(double dtSeconds) {
    // --- Angle ---
    double angleError = wrapToPi(m_targetAngle.getRadians() - m_currentAngle.getRadians());
    double maxStep = m_maxAngleRadPerSecond * dtSeconds;
    double angleStep = clamp(angleError, -maxStep, maxStep);
    m_currentAngle = new Rotation2d(m_currentAngle.getRadians() + angleStep);

    // --- Speed ---
    double speedError = m_targetSpeedMetersPerSecond - m_currentSpeedMetersPerSecond;
    double speedMaxStep = m_maxSpeedMetersPerSecondPerSecond * dtSeconds;
    double speedStep = clamp(speedError, -speedMaxStep, speedMaxStep);
    m_currentSpeedMetersPerSecond += speedStep;
  }

  public Rotation2d getAngle() {
    return m_currentAngle;
  }

  public double getSpeedMetersPerSecond() {
    return m_currentSpeedMetersPerSecond;
  }

  // Minimal state class to avoid extra dependencies.
  public record SwerveModuleState(Rotation2d angle, double speedMetersPerSecond) {}

  private static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }

  private static double wrapToPi(double radians) {
    // Wrap to [-pi, pi)
    double wrapped = (radians + Math.PI) % (2.0 * Math.PI);
    if (wrapped < 0) {
      wrapped += 2.0 * Math.PI;
    }
    return wrapped - Math.PI;
  }
}

