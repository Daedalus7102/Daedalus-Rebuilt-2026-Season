package frc.robot.subsystems.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SpoolShooter extends Command {
    private final ShooterSubsystem shooter;
    
    private final double distance;
    
    public SpoolShooter(ShooterSubsystem shooter, double distance) {
        this.distance = distance;
        this.shooter = shooter;
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        shooter.aim(distance);
        shooter.setMeasuredRPM(distance);
    }

    @Override
    public void end(boolean _i) {
        shooter.disable();
    }
}
