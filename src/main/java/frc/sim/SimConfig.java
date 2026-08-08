package frc.sim;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

public class SimConfig {
    public static final int rightDriveLead_id = 1;
    public static final int rightDriveFollow_id = 2;
    public static final int leftDriveLead_id = 3;
    public static final int leftDriveFollow_id = 4;
    public static final double MASS_KG = 25;
    public static final double GEARING = 1;
    public static final Distance WHEEL_RADIUS = Inches.of(2.5);
    public static final Distance TRACK_WIDTH = Inches.of(24);
    public static final double ROBOT_MOI = 8;
}
