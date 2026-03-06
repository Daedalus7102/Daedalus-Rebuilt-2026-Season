package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
	private final SparkFlex shooterMotor1;
	private final SparkFlex shooterMotor2;
	private final SparkFlex shooterMotor3;
	private final SparkFlex hoodMotor;

	private final SparkFlexConfig shooterMotorConfig1;
	private final SparkFlexConfig shooterMotorConfig2;
	private final SparkFlexConfig shooterMotorConfig3;
	private final SparkFlexConfig hoodMotorConfig;

	public ShooterSubsystem() {
		shooterMotor1 = new SparkFlex(ShooterConstants.shootMotor1ID, SparkLowLevel.MotorType.kBrushless);
		shooterMotor2 = new SparkFlex(ShooterConstants.shootMotor2ID, SparkLowLevel.MotorType.kBrushless);
		shooterMotor3 = new SparkFlex(ShooterConstants.shootMotor3ID, SparkLowLevel.MotorType.kBrushless);

		shooterMotorConfig1 = new SparkFlexConfig();
		shooterMotorConfig2 = new SparkFlexConfig();
		shooterMotorConfig3 = new SparkFlexConfig();

		shooterMotorConfig1.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.pid(ShooterConstants.shooterP, ShooterConstants.shooterI, ShooterConstants.shooterD);
		shooterMotorConfig1.closedLoop.feedForward.kV(ShooterConstants.shooterKV);

		shooterMotorConfig2.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.pid(ShooterConstants.shooterP, ShooterConstants.shooterI, ShooterConstants.shooterD);
		shooterMotorConfig2.closedLoop.feedForward.kV(ShooterConstants.shooterKV);

		shooterMotorConfig3.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.pid(ShooterConstants.shooterP, ShooterConstants.shooterI, ShooterConstants.shooterD);
		shooterMotorConfig3.closedLoop.feedForward.kV(ShooterConstants.shooterKV);

		shooterMotorConfig1.idleMode(SparkBaseConfig.IdleMode.kCoast);
		shooterMotorConfig2.idleMode(SparkBaseConfig.IdleMode.kCoast);
		shooterMotorConfig3.idleMode(SparkBaseConfig.IdleMode.kCoast);

		shooterMotor1.configure(shooterMotorConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		shooterMotor2.configure(shooterMotorConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		shooterMotor3.configure(shooterMotorConfig3, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		hoodMotor = new SparkFlex(ShooterConstants.hoodMotorID, SparkLowLevel.MotorType.kBrushless);
		hoodMotorConfig = new SparkFlexConfig();
		hoodMotorConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kDetachedAbsoluteEncoder)
				.pid(0.0001, 0, 0);
		hoodMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
		hoodMotor.configure(hoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	public void disable() {
		shooterMotor1.stopMotor();
		shooterMotor2.stopMotor();
		shooterMotor3.stopMotor();
	}

	public void setShooterRPM(double rpm) {
		double clampedRPM = Math.max(0.0, rpm);
		shooterMotor1.getClosedLoopController().setSetpoint(clampedRPM, ControlType.kVelocity);
		shooterMotor2.getClosedLoopController().setSetpoint(clampedRPM, ControlType.kVelocity);
		shooterMotor3.getClosedLoopController().setSetpoint(clampedRPM, ControlType.kVelocity);
	}

	public void setMeasuredRPM(double distance) {
		double rpm = LookUpTable.getPoint(distance).rpm();
		setShooterRPM(rpm);
	}

	public double getShooterRPM() {
		double rpm1 = shooterMotor1.getEncoder().getVelocity();
		double rpm2 = shooterMotor2.getEncoder().getVelocity();
		double rpm3 = shooterMotor3.getEncoder().getVelocity();
		return (rpm1 + rpm2 + rpm3) / 3.0;
	}

	public boolean isAtTargetRPM(double toleranceRPM) {
		double targetRPM = shooterMotor1.getClosedLoopController().getSetpoint();
		return Math.abs(getShooterRPM() - targetRPM) <= toleranceRPM;
	}

	public void setHoodAngle(double angle) {
		hoodMotor.getClosedLoopController().setSetpoint(
				Math.max(Math.min(angle, ShooterConstants.maxHoodAngle), ShooterConstants.minHoodAngle),
				SparkBase.ControlType.kPosition
		);
	}

	public void setMeasuredHoodAngle(double distance) {
		double angle = LookUpTable.getPoint(distance).angle();
		setHoodAngle(angle);
	}

	public double getHoodAngle() {
		return hoodMotor.getExternalEncoder().getPosition();
	}

	public void aim(double distance) {
		LookUpTable.DataPoint dPoint = LookUpTable.getPoint(distance);
		setHoodAngle(dPoint.angle());
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("HoodAngle", getHoodAngle());
		SmartDashboard.putNumber("HoodTargetAngle", hoodMotor.getClosedLoopController().getSetpoint());

		SmartDashboard.putNumber("ShooterRPM", getShooterRPM());
		SmartDashboard.putNumber("ShooterTargetRPM", shooterMotor1.getClosedLoopController().getSetpoint());
		SmartDashboard.putBoolean("ShooterAtSpeed", isAtTargetRPM(ShooterConstants.shooterReadyToleranceRPM));
	}
}
