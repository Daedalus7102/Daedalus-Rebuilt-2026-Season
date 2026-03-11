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
        public static final double maxSpeed = 3; // Maximum drive speed in meters per second
		public static final double maxTurnRate = 2; // max turn rate in radians per second
	}

	public static class IntakeConstants {
		// Motor IDs
		public static final int kRollerMotorID = 12;
		public static final int kRotateMotorID = 10;
		public static final int kRotateFollowerMotorID = 11;
	}

	public static class ShooterConstants {
		public static final int shootMotor1ID = 23;
		public static final int shootMotor2ID = 24;
		public static final int shootMotor3ID = 25;
		public static final int hoodMotorID = 22;
		public static final int indexerMotorID = 21;
		public static final int feederMotorID = 20;

		public static final double maxHoodAngle = 30;
		public static final double minHoodAngle = 10;

		// Shooter motor configuration
		public static final int shootCurrentLimit = 35;
		public static final double shootRampRate = 0.05;
		public static final double voltageCompensation = 12;

		// Shooter closed-loop (Spark velocity control in RPM)
		// 1:1
		public static final double shooterP = 0.0025;
		public static final double shooterI = 0.0;
		public static final double shooterD = 0.002;
		public static final double shooterKV = 0.0004;

		// Shooter hood closed-loop (Position control through absolute encoder)
		public static final double hoodP = 0.032;
		public static final double hoodI = 0;
		public static final double hoodD = 0;
		public static final double hoodKV = 0;
		public static final int hoodCurrentLimit = 30;
		public static final double hoodRampRate = 0.5;
		public static final double hoodClosedLoopMinOutput = -0.4;
		public static final double hoodClosedLoopMaxOutput = 0.4;
		public static final double hoodReadyToleranceDeg = 1.0;

		public static final double shooterTargetRPM = 5000;
		public static final double shooterReadyToleranceRPM = 200;
		public static final double shooterMinRPM = 3000;
		public static final double unclogRPM = -4000;

		// Feeder & Indexer
		public static final double feederSpeed = 0.2;
		public static final double indexerSpeed = 0.8;

		public static final double feedingShooterRPM = shooterTargetRPM;
		public static final double feedingHoodAngle = 0;
	}

	public static class VisionConstants {
		// placeholders
		public static final Matrix<N3, N1> singleTagDeviation = VecBuilder.fill(4, 4, 8);
		public static final Matrix<N3, N1> multiTagDeviation = VecBuilder.fill(0.5, 0.5, 1);
	}
}