package frc.robot.subsystems.shooter;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.led.LEDState;

public class ShooterSubsystem extends SubsystemBase {
	private final SparkFlex shooterMotor1;
	private final SparkFlex shooterMotor2;
	private final SparkFlex shooterMotor3;
	private final SparkFlex hoodMotor;

	private final SparkFlexConfig shooterMotorConfig;
	private final SparkFlexConfig hoodMotorConfig;

	public ShooterSubsystem() {
		shooterMotor1 = new SparkFlex(ShooterConstants.shootMotor1ID, SparkLowLevel.MotorType.kBrushless);
		shooterMotor2 = new SparkFlex(ShooterConstants.shootMotor2ID, SparkLowLevel.MotorType.kBrushless);
		shooterMotor3 = new SparkFlex(ShooterConstants.shootMotor3ID, SparkLowLevel.MotorType.kBrushless);
		shooterMotorConfig = new SparkFlexConfig();
		shooterMotorConfig.closedLoop
								.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
								.pid(0.01, 0, 0);

		shooterMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
		shooterMotorConfig.follow(ShooterConstants.shootMotor1ID);
		shooterMotor1.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		shooterMotor2.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		shooterMotor3.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
		RobotContainer.leds.set(LEDState.OFF);
	}

    public void setShooterRPM(double rpm) {
		shooterMotor1.getClosedLoopController().setSetpoint(rpm, SparkBase.ControlType.kVelocity);
		RobotContainer.leds.set(LEDState.SHOOT);
    }

	// using LUT
	public void setMeasuredRPM(double distance) {
		double rpm = LookUpTable.getPoint(distance).rpm();
		setShooterRPM(rpm);
	}

	public void setHoodAngle(double angle) {
		hoodMotor.getClosedLoopController().setSetpoint(
				Math.max(Math.min(angle, ShooterConstants.maxHoodAngle), ShooterConstants.minHoodAngle),
				SparkBase.ControlType.kPosition
		);
	}

	// using LUT
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

		SmartDashboard.putNumber("ShooterRPM", shooterMotor1.getEncoder().getVelocity());
		SmartDashboard.putNumber("ShooterTargetRPM", shooterMotor1.getClosedLoopController().getSetpoint());
    }
}
