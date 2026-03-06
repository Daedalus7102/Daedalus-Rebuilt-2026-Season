package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    // Drive Constants
    public static class SwerveConstants {
        // Chassis Constraints
        private static final double kWheelToWheelDistance = 0.56; // Distance between wheels in meters
        // Module Constants
        public static final double kWheelDiameter = 0.101; // Wheel diameter in meters
        public static final double kWheelCircumference = Math.PI * kWheelDiameter;
        public static final double kDriveMaxSpeed = 3.6; // Maximum drive speed in meters per second
        public static final double kDriveMaxAcc = 0.6; // Maximum drive acceleration in meters per second squared
        public static final double kDriveGearRatio = 1d/6.75d; // 5.36;
        public static final double kTurnMaxSpeed = Math.PI*2; // Maximum turn speed in radians per second
        public static final double kTurnMaxAcc = 0.15; // Maximum turn acceleration in degrees per second squared
        public static final double kTurnGearRatio = 150d/7d;
        // Drive encoder
        public static final double kDriveVelocityFactor = (kDriveGearRatio * kWheelCircumference) / 60.0; // m/s per RPM
        // Turning encoder
        public static final double kTurnPositionFactor = 360.0 / kTurnGearRatio; // deg / motor rot
        public static final double kTurnVelocityFactor = kTurnPositionFactor / 60.0; // deg/s per RPM
        // Swerve Drive Kinematics
        public final static SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelToWheelDistance / 2d, kWheelToWheelDistance / 2d), // Frront Left Module
                new Translation2d(kWheelToWheelDistance / 2d, -kWheelToWheelDistance / 2d), // Front Right Module
                new Translation2d(-kWheelToWheelDistance / 2d, kWheelToWheelDistance / 2d), // Back Left Module
                new Translation2d(-kWheelToWheelDistance / 2d, -kWheelToWheelDistance / 2d) // Back Right Module
        );
        // Swerve Modules

        //    .---.         .---.   Forward = 0°
        //    | 1 |▩▩▩▩▩▩▩▩▩| 2 |
        //    '---'         '---'    (x+)|
        //      █     /°\     █          |
        //      █      |      █   (y+)---|---(y-)
        //      █      |      █          |
        //    .---.         .---.        |(x-)
        //    | 3 |▩▩▩▩▩▩▩▩▩| 4 |        
        //    '---'         '---'

        public static final int kFrontLeftDriveMotorID = 1;
        public static final int kFrontLeftTurnMotorID = 2;
        public static final int kFrontLeftCANcoderID = 1;
        public static final double kFrontLeftCANcoderOffset = -0.311035;
        public static final boolean kFrontLeftDriveInverted = false;

        public static final int kFrontRightDriveMotorID = 3;
        public static final int kFrontRightTurnMotorID = 4;
        public static final int kFrontRightCANcoderID = 2;
        public static final double kFrontRightCANcoderOffset = 0.305664;
        public static final boolean kFrontRightDriveInverted = true;

        public static final int kBackLeftDriveMotorID = 5;
        public static final int kBackLeftTurnMotorID = 6;
        public static final int kBackLeftCANcoderID = 3;
        public static final double kBackLeftCANcoderOffset = 0.433350;
        public static final boolean kBackLeftDriveInverted = false;

        public static final int kBackRightDriveMotorID = 7;
        public static final int kBackRightTurnMotorID = 8;
        public static final int kBackRightCANcoderID = 4;
        public static final double kBackRightCANcoderOffset = 0.055908;
        public static final boolean kBackRightDriveInverted = true;

        // Drive Motor PID Constants
        public static final double kDriveP = 0.015, kDriveI = 0.0, kDriveD = 0.0, kDriveFF = 0.2;
        // Turning Motor PID Constants
        public static final double kTurnP = 0.01, kTurnI = 0.0, kTurnD = 0.0;
        // Drive Motor Configuration
        public static final int kDriveCurrentLimitA = 35;
        public static final double kDriveVoltageComp = 12.0;
        public static final double kDriveOpenLoopRamp = 0.5;
        public static final double kDriveClosedLoopRamp = 0.5;
        // Turning Motor Configuration
        public static final int kTurnCurrentLimitA = 25;
        public static final double kTurnVoltageComp = 12.0;
        public static final double kTurnOpenLoopRamp = 0.15;
        public static final double kTurnClosedLoopRamp = 0.15;
        // CAN bus
        public static final String kCANbus = "Drivetrain";
        // Pigeon ID
        public static final int kPigeonID = 0;
    }

	public static class ShooterConstants {
		public static final int shootMotor1ID = 23;
		public static final int shootMotor2ID = 24;
		public static final int shootMotor3ID = 25;
		public static final int hoodMotorID = 22;
		public static final int indexerMotorID = 21;
		public static final int feederMotorID = 20;

		public static final double maxHoodAngle = 0.064;
		public static final double minHoodAngle = 0.0;

		// Shooter closed-loop (Spark velocity control in RPM)
		// 1:1
		public static final double shooterP = 0.0002;
		public static final double shooterI = 0.0;
		public static final double shooterD = 0.0;
		public static final double shooterKV = 0.00015;

		public static final double shooterTargetRPM = 4500;
		public static final double shooterReadyToleranceRPM = 150;

        // Feeder & Indexer
        public static final double feederSpeed = 0.70;
        public static final double indexerSpeed = 0.70;

		public static final double feedingShooterRPM = shooterTargetRPM;
        public static final double feedingHoodAngle = 0;
	}
}