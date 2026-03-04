package frc.robot.subsystems.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class TestShooterCommand extends Command {
    private final ShooterSubsystem Shooter;
    private final FeederSubsystem Feeder;
    private final double distance;
    
    public TestShooterCommand(ShooterSubsystem Shooter, FeederSubsystem Feeder, double distance) {
        this.distance = distance;
        this.Shooter = Shooter;
        this.Feeder = Feeder;
        addRequirements(Shooter, Feeder);
    }
    
    @Override
    public void execute() {
        Feeder.enable();
        Shooter.setMeasuredRPM(1);
    }

    @Override
    public void end(boolean _i) {
        Feeder.disable();
        Shooter.setShooterRPM(0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
