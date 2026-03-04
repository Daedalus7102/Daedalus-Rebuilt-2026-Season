package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
	private final SparkFlex indexerMotor;
	private final SparkFlex feederMotor;

	private final SparkFlexConfig indexerMotorConfig;
	private final SparkFlexConfig feederMotorConfig;

	public FeederSubsystem() {
		indexerMotor = new SparkFlex(Constants.ShooterConstants.indexerMotorID, SparkLowLevel.MotorType.kBrushless);
		indexerMotorConfig = new SparkFlexConfig();
		indexerMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
		indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		feederMotor = new SparkFlex(Constants.ShooterConstants.feederMotorID, SparkLowLevel.MotorType.kBrushless);
		feederMotorConfig = new SparkFlexConfig();
		feederMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
		feederMotor.configure(feederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	public void setSpeed(double speed) {
		indexerMotor.set(speed);
		feederMotor.set(speed);
	}

	@Override
    public void periodic() {
		SmartDashboard.putNumber("FeederSpeed", feederMotor.get());
		SmartDashboard.putNumber("IndexerSpeed", indexerMotor.get());
    }
}
