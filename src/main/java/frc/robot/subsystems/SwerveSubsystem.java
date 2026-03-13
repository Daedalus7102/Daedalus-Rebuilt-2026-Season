package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.camera.Vision;
import frc.robot.tools.AllianceTargetPoses;

import org.photonvision.EstimatedRobotPose;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.List;
import java.util.function.DoubleSupplier;

public class SwerveSubsystem extends SubsystemBase {

	/**
	 * <ul>
	 *   <li>{@link DriveMode#NORMAL}: normal field-oriented driving.</li>
	 *   <li>{@link DriveMode#AUTO_HUB}: locks rotation to point to the hub, field-oriented.</li>
	 *   <li>{@link DriveMode#AUTO_TEAM}: locks rotation to point to 0 degrees, field-oriented.</li>
	 * </ul>
	 */
	public enum DriveMode {
		NORMAL,
		AUTO_HUB,
		AUTO_TEAM,
		AUTO_TRENCH
	}

	private final DoubleSupplier joystickX, joystickY, joystickRotation;
	private final DoubleSupplier dPadX, dPadY;
	private DriveMode driveMode = DriveMode.NORMAL;
	private Translation2d hubPos = new Translation2d(0, 0);
	private final double AUTO_HUB_INPUT_SCALE = 0.2;
	private final double DPAD_SCALE = 0.2;
	public double inputMultiplier = 1;
	private boolean scaleInput = false;

	private final Vision vision;
	public final SwerveDrive swerveDrive;

	public SwerveSubsystem(DoubleSupplier joystickX, DoubleSupplier joystickY, DoubleSupplier joystickRotation, DoubleSupplier dPadX, DoubleSupplier dPadY) {
		boolean blueAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue;
    	Pose2d startingPose = !blueAlliance ? new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0))
                                       : new Pose2d(new Translation2d(Meter.of(16), Meter.of(4)),Rotation2d.fromDegrees(180));
		this.dPadX = dPadX;
		this.dPadY = dPadY;
		try {
			File configDir = new File(Filesystem.getDeployDirectory(), "swerve");
			swerveDrive = new SwerveParser(configDir).createSwerveDrive(Constants.SwerveConstants.maxSpeed, startingPose);
		} catch (Exception e) {
			throw new RuntimeException(e);
		}
		this.joystickX = joystickX;
		this.joystickY = joystickY;
		this.joystickRotation = joystickRotation;
		this.vision = new Vision(this);

		try {
			RobotConfig config = RobotConfig.fromGUISettings();
			AutoBuilder.configure(
					swerveDrive::getPose,
					swerveDrive::resetOdometry,
					swerveDrive::getRobotVelocity,
					chassisSpeeds -> swerveDrive.drive(chassisSpeeds),
					new PPHolonomicDriveController(
							new PIDConstants(5.0, 0.0, 0.0),
							new PIDConstants(5.0, 0.0, 0.0)
					),
					config,
					() -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
							== DriverStation.Alliance.Red,
					this
			);
		} catch (Exception e) {
			DriverStation.reportError(
					"Failed to configure PathPlanner AutoBuilder: " + e.getMessage(),
					e.getStackTrace());
		}
	}

	@Override
	public void periodic() {
		vision.updatePose();

		Translation2d translation;
		double rotation = getControllerRotation();

		if (usingDPads()) {
			translation = getDPadTranslation();
		} else {
			translation = getControllerTranslation();
		}

		switch (driveMode) {
			case NORMAL:
				drive(translation, rotation, !usingDPads());
				break;
			case AUTO_HUB:
				driveAutoHub(translation);
				break;
			case AUTO_TEAM:
				driveAutoTeam(translation);
				break;
			case AUTO_TRENCH:
				driveTargetAngle(translation, 0);
				break;
		}
	}

	public void setScaleInput(boolean scaleInput) {
		this.scaleInput = scaleInput;
	}

	public double getScaleInputValue() {
		return scaleInput ? inputMultiplier : 1;
	}

	public boolean usingDPads() {
		return dPadX.getAsDouble() != 0 || dPadY.getAsDouble() != 0;
	}

	/*
	SmartDashboard.putNumber("Translation X", translation.getX());
		SmartDashboard.putNumber("Translation Y", translation.getY());
		for (Map.Entry<String, SwerveModule> m : swerveDrive.getModuleMap().entrySet()) {
			SmartDashboard.putNumber(m.getKey() + " RPM", m.getValue().getDriveMotor().getVelocity());
		}
		SmartDashboard.putNumber("Rotation", rotation);
		SmartDashboard.putString("Mode", driveMode.name());
	 */

	public void setFieldLine(String objectName, Translation2d startPoint, Translation2d endPoint) {
		Translation2d delta = endPoint.minus(startPoint);
		if (delta.getNorm() < 1e-6) {
			swerveDrive.field.getObject(objectName).setPoses(new Pose2d(startPoint, Rotation2d.kZero));
			return;
		}

		Rotation2d heading = delta.getAngle();
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
				new Pose2d(startPoint, heading),
				List.of(),
				new Pose2d(endPoint, heading),
				new TrajectoryConfig(2.0, 2.0));

		swerveDrive.field.getObject(objectName).setTrajectory(trajectory);
	}

	/**
	 * Convenience wrapper around swerveDrive.getPose().
	 */
	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	/**
	 * Aims the swerve at the given field point and switches to AUTO_HUB mode.
	 * Returns an instant Command so it can be used as the first step in a sequence.
	 *
	 * @param target the field {@link Translation2d} to aim at.
	 * @return an instant {@link Command} that sets hub position and activates AUTO_HUB mode.
	 */
	public Command enableAutoAimAtPoint(Translation2d target) {
		return Commands.runOnce(() -> {
			setHubPos(target);
			setMode(DriveMode.AUTO_HUB);
		}, this);
	}

	/**
	 * Immediately returns the swerve to {@link DriveMode#NORMAL}.
	 * Intended for use in finallyDo() blocks.
	 */
	public void disableAutoAimNow() {
		resetMode();
	}

	private Translation2d getControllerTranslation() {
		Translation2d translation = new Translation2d(
				MathUtil.applyDeadband(-joystickY.getAsDouble(), 0.1),
				MathUtil.applyDeadband(-joystickX.getAsDouble(), 0.1)
		);

		if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
			translation = translation.rotateBy(Rotation2d.k180deg);
		}

		return translation.times(getScaleInputValue() * Constants.SwerveConstants.maxSpeed);
	}

	private Translation2d getDPadTranslation() {
		Translation2d translation = new Translation2d(
				dPadX.getAsDouble(),
				dPadY.getAsDouble()
		);

		return translation.times(getScaleInputValue() * Constants.SwerveConstants.maxSpeed * DPAD_SCALE);
	}

	private double getControllerRotation() {
		return MathUtil.applyDeadband(-joystickRotation.getAsDouble(), 0.1) *
				Constants.SwerveConstants.maxTurnRate * getScaleInputValue();
	}

	public void resetOdometryRotation() {
		swerveDrive.resetOdometry(new Pose2d(
				swerveDrive.getPose().getTranslation(), new Rotation2d(0)
		));
	}

	/**
	 * Sets the position of the hub. When in {@link DriveMode#AUTO_HUB} mode,
	 * the swerve will aim at this position compensating for velocity.
	 *
	 * @param pos {@link Translation2d} position of the hub.
	 */
	public void setHubPos(Translation2d pos) {
		hubPos = pos;
	}

	/**
	 * Sets the drive mode.
	 *
	 * @param mode {@link DriveMode} the mode.
	 */
	public void setMode(DriveMode mode) {
		driveMode = mode;
	}

	/**
	 * Sets the drive mode to {@link DriveMode#NORMAL}.
	 */
	public void resetMode() {
		driveMode = DriveMode.NORMAL;
	}

	public void addVisionMeasurement(EstimatedRobotPose pose, Matrix<N3, N1> deviation) {
		if (driveMode == DriveMode.AUTO_HUB)
			swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, deviation);
	}

	private void driveBotRelative(Translation2d translation, double rotation) {
		drive(translation, rotation, false);
	}

	private void driveAutoTeam(Translation2d translation) {
		driveTargetAngle(translation, Math.PI);
	}

	private void driveAutoHub(Translation2d translation) {
		Pose2d pos = AllianceTargetPoses.getTowerPoseForCurrentAlliance();
		driveTargetAngle(translation.times(AUTO_HUB_INPUT_SCALE), getRotationToPoint(pos.getTranslation()));
	}

	private double getRotationToPoint(Translation2d target) {
		Pose2d pose = swerveDrive.getPose();
		Translation2d delta = target.minus(pose.getTranslation());
		return Math.atan2(delta.getY(), delta.getX());
	}

	private void driveTargetAngle(Translation2d translation, double targetAngle) {
		ChassisSpeeds target = swerveDrive.swerveController.getRawTargetSpeeds(
				translation.getX(), translation.getY(),
				targetAngle,
				swerveDrive.getOdometryHeading().getRadians()
		);

		ChassisSpeeds limited = new ChassisSpeeds(
				target.vxMetersPerSecond,
				target.vyMetersPerSecond,
				Math.min(Math.max(target.omegaRadiansPerSecond, -Constants.SwerveConstants.maxTurnRate), Constants.SwerveConstants.maxTurnRate)
		);

		swerveDrive.driveFieldOriented(limited);
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