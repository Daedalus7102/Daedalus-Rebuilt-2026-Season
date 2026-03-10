package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AimCommand extends Command{

    private ShooterSubsystem shooterSubsystem;
    private double distance;

    public AimCommand(ShooterSubsystem shooterSubsystem, double distance) {
        this.shooterSubsystem = shooterSubsystem;
        this.distance = distance;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.aim(distance);
    }

    @Override
    public void end(boolean _i) {
        shooterSubsystem.disable();
    }
}
