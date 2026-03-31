package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class FeedexerSubsystem extends SubsystemBase {
	private final SparkMax feedexerPrimaryMotor;
	private final SparkFlex feedexerSecondaryMotor;

	private final SparkMaxConfig feedexerPrimaryMotorConfig;
	private final SparkFlexConfig feedexerSecondaryMotorConfig;

	public FeedexerSubsystem() {
		feedexerPrimaryMotor = new SparkMax(
				ShooterConstants.feedexerPrimaryMotorID,
				SparkLowLevel.MotorType.kBrushless
		);
		feedexerPrimaryMotorConfig = new SparkMaxConfig();
		feedexerPrimaryMotorConfig
				.idleMode(SparkBaseConfig.IdleMode.kCoast)
				.smartCurrentLimit(35)
				.voltageCompensation(12)
				.inverted(ShooterConstants.feedexerPrimaryInverted);
		feedexerPrimaryMotor.configure(
				feedexerPrimaryMotorConfig,
				ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters
		);

		feedexerSecondaryMotor = new SparkFlex(
				ShooterConstants.feedexerSecondaryMotorID,
				SparkLowLevel.MotorType.kBrushless
		);
		feedexerSecondaryMotorConfig = new SparkFlexConfig();
		feedexerSecondaryMotorConfig
				.idleMode(SparkBaseConfig.IdleMode.kCoast)
				.smartCurrentLimit(35)
				.voltageCompensation(12)
				.inverted(ShooterConstants.feedexerSecondaryInverted);
		feedexerSecondaryMotor.configure(
				feedexerSecondaryMotorConfig,
				ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters
		);
	}

	public void setSpeed(double speed) {
		feedexerPrimaryMotor.set(speed);
		feedexerSecondaryMotor.set(speed);
	}

	public void enable() {
		setSpeed(ShooterConstants.feedexerSpeed);
	}

	public void disable() {
		feedexerPrimaryMotor.stopMotor();
		feedexerSecondaryMotor.stopMotor();
	}

	public void unclog() {
		setSpeed(-0.6);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("FeedexerPrimarySpeed", feedexerPrimaryMotor.get());
		SmartDashboard.putNumber("FeedexerSecondarySpeed", feedexerSecondaryMotor.get());
	}
}