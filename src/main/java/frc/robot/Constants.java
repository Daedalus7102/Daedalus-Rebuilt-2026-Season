package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    public static class RuntimeConstants {
        /**
         * When true, selected parameters can be changed live from SmartDashboard.
         * Keep this true for current behavior parity; set false for competition lock-down.
         */
        public static final boolean kEnableLiveTuning = true;
    }

    // Drive Constants
    public static class SwerveConstants {
        public static final double kDriveMaxSpeed = 3.6; // Maximum drive speed in meters per second
    }
}