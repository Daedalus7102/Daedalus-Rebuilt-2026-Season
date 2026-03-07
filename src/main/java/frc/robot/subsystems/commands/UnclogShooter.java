package frc.robot.subsystems.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class UnclogShooter extends Command {
    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;

    public UnclogShooter(ShooterSubsystem shooter, FeederSubsystem feeder) {
        this.shooter = shooter;
        this.feeder = feeder;
        addRequirements(shooter, feeder);
    }
    
    @Override
    public void initialize() {
        shooter.unclog();
        feeder.unclog();
    }

    @Override
    public void end(boolean _i) {
        shooter.disable();
        feeder.disable();
    }
}