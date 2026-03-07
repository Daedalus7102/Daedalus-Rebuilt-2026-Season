package frc.robot.subsystems.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;


public class FeedShooter extends Command {
    
    private final FeederSubsystem feeder;
    private final ShooterSubsystem shooter; // <- para hacer lo de blocking
	private final boolean block;
    
    public FeedShooter(FeederSubsystem feeder, ShooterSubsystem shooter, boolean block) {
        this.shooter = shooter;
        this.feeder = feeder;
        addRequirements(feeder);
		this.block = block;
    }

	@Override
	public void execute() {
		if ((shooter.isAtTargetRPM(Constants.ShooterConstants.shooterReadyToleranceRPM) && shooter.getTargetRPM() > Constants.ShooterConstants.shooterMinRPM) || !block) {
			feeder.enable();
		} else {
			feeder.disable();
		}
	}

	@Override
    public void end(boolean _i) {
        feeder.disable();
    }
}
