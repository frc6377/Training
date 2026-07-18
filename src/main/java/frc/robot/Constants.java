package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    public static class Swerve {
        public static final double kTrackWidth = 12.9; // meters
        public static final double kWheelBase = 12.9; // meters

        // Module locations
        public static final Translation2d kFrontLeftLocation = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
        public static final Translation2d kFrontRightLocation = new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
        public static final Translation2d kBackLeftLocation = new Translation2d(-kWheelBase / 2, kTrackWidth / 2);
        public static final Translation2d kBackRightLocation = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);
    }
}