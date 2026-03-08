package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

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

	public static class VisionConstants {
		// placeholders
		public static final Matrix<N3, N1> singleTagDeviation = VecBuilder.fill(4, 4, 8);
		public static final Matrix<N3, N1> multiTagDeviation = VecBuilder.fill(0.5, 0.5, 1);
	}
}