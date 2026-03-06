package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import java.io.File;
import java.util.function.DoubleSupplier;

public class SwerveSubsystem extends SubsystemBase {

	public enum DriveMode {
		FIELD_RELATIVE,
		BOT_RELATIVE,
		AUTO_HUB,
		AUTO_TEAM
	}

	private DoubleSupplier joystickX, joystickY, joystickRotation;
	private DriveMode driveMode = DriveMode.FIELD_RELATIVE;
	private Translation2d hubPos = new Translation2d(0, 0);

	private final SwerveDrive swerveDrive;

	public SwerveSubsystem(DoubleSupplier joystickX, DoubleSupplier joystickY, DoubleSupplier joystickRotation) {
		try {
			File configDir = new File(Filesystem.getDeployDirectory(), "swerve");
			swerveDrive = new SwerveParser(configDir).createSwerveDrive(Constants.SwerveConstants.kDriveMaxSpeed);
		} catch (Exception e) {
			throw new RuntimeException(e);
		}
		this.joystickX = joystickX;
		this.joystickY = joystickY;
		this.joystickRotation = joystickRotation;
	}

	@Override
	public void periodic() {
		Translation2d translation = new Translation2d(joystickX.getAsDouble(), joystickY.getAsDouble());
		double rotation = joystickRotation.getAsDouble();
		switch (driveMode) {
			case FIELD_RELATIVE:
				driveFieldRelative(translation, rotation);
				break;
			case BOT_RELATIVE:
				driveBotRelative(translation, rotation);
				break;
			case AUTO_HUB:
				driveAutoHub(translation);
				break;
			case AUTO_TEAM:
				driveAutoTeam(translation);
		}
	}

	public void setHubPos(Translation2d pos) {
		hubPos = pos;
	}

	public void setMode(DriveMode mode) {
		driveMode = mode;
	}

	public void resetMode() {
		driveMode = DriveMode.FIELD_RELATIVE;
	}

	private void driveFieldRelative(Translation2d translation, double rotation) {
		drive(translation, rotation, true);
	}

	private void driveBotRelative(Translation2d translation, double rotation) {
		drive(translation, rotation, false);
	}

	private void driveAutoTeam(Translation2d translation) {
		driveTargetAngle(translation, 180);
	}

	private void driveAutoHub(Translation2d translation) {
		Translation2d compensatedPos = new Translation2d( // todo adjust subtraction depending on airtime or something
				hubPos.getX() - swerveDrive.getFieldVelocity().vxMetersPerSecond,
				hubPos.getY() - swerveDrive.getFieldVelocity().vyMetersPerSecond
		);
		driveTargetAngle(translation, getRotationToPoint(compensatedPos));
	}

	private double getRotationToPoint(Translation2d target) {
		Pose2d pose = swerveDrive.getPose();
		Translation2d delta = target.minus(pose.getTranslation());
		return Math.atan2(delta.getY(), delta.getX());
	}

	private void driveTargetAngle(Translation2d translation, double targetAngle) {
		swerveDrive.driveFieldOriented(
				swerveDrive.swerveController.getTargetSpeeds(
						translation.getX(), translation.getY(),
						targetAngle,
						swerveDrive.getOdometryHeading().getRadians(),
						Constants.SwerveConstants.kDriveMaxSpeed
				)
		);
	}

	private void drive(Translation2d translation, double rotation, boolean fieldRelative) {
		swerveDrive.drive(
				translation,
				rotation,
				fieldRelative,
				false
		);
	}
}
