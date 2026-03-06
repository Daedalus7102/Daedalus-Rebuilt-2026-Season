package frc.robot.subsystems.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;


public class FeedShooter extends Command {
    
    private final FeederSubsystem feeder;
    private final ShooterSubsystem shooter; // <- para hacer lo de blocking
    
    public FeedShooter(FeederSubsystem feeder, ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.feeder = feeder;
        addRequirements(feeder);
    }
    
    @Override
    public void initialize() {
        feeder.enable();
    }

    @Override
    public void end(boolean _i) {
        feeder.disable();
    }
}
