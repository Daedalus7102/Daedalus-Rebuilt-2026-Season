package frc.robot.subsystems.shooter;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
	private final SparkFlex shooterMotor1;
	private final SparkFlex shooterMotor2;
	private final SparkFlex shooterMotor3;
	private final SparkFlex hoodMotor;
	private final SparkFlex indexerMotor;
	private final SparkFlex feederMotor;

	private final SparkFlexConfig shooterMotorConfig;
	private final SparkFlexConfig hoodMotorConfig;
	private final SparkFlexConfig indexerMotorConfig;
	private final SparkFlexConfig feederMotorConfig;

	public ShooterSubsystem() {
		shooterMotor1 = new SparkFlex(ShooterConstants.shootMotor1ID, null);
		shooterMotor2 = new SparkFlex(ShooterConstants.shootMotor2ID, null);
		shooterMotor3 = new SparkFlex(ShooterConstants.shootMotor3ID, null);
		shooterMotorConfig = new SparkFlexConfig();
		shooterMotorConfig.closedLoop
								.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
								.pid(0.01, 0, 0);

		shooterMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
		shooterMotorConfig.follow(ShooterConstants.shootMotor1ID);
		shooterMotor1.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		shooterMotor2.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		shooterMotor3.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		hoodMotor = new SparkFlex(ShooterConstants.hoodMotorID, null);
		hoodMotorConfig = new SparkFlexConfig();
		hoodMotorConfig.closedLoop
								.feedbackSensor(FeedbackSensor.kDetachedAbsoluteEncoder)
								.pid(0.0001, 0, 0);

		hoodMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
		hoodMotor.configure(hoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		indexerMotor = new SparkFlex(ShooterConstants.indexerMotorID, null);
		indexerMotorConfig = new SparkFlexConfig();
		indexerMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
		indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		feederMotor = new SparkFlex(ShooterConstants.feederMotorID, null);
		feederMotorConfig = new SparkFlexConfig();
		feederMotorConfig.idleMode(IdleMode.kCoast);
		feederMotor.configure(feederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

    public void setShooterRPM(double rpm) {
		shooterMotor1.getClosedLoopController().setSetpoint(rpm, SparkBase.ControlType.kVelocity);
    }

	public void setHoodAngle(double angle) {
		hoodMotor.getClosedLoopController().setSetpoint(
				Math.max(Math.min(angle, ShooterConstants.maxHoodAngle), ShooterConstants.minHoodAngle),
				SparkBase.ControlType.kPosition
		);
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
        // Shuffleboard stuff
    }
}
