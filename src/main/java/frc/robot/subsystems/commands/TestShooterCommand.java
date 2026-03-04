package frc.robot.subsystems.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class TestShooterCommand extends Command {
    private final ShooterSubsystem Shooter;
    private final double rpm;
    
    public TestShooterCommand(ShooterSubsystem Shooter, double RPM) {
        this.rpm = RPM;
        this.Shooter = Shooter;
        addRequirements(Shooter);
    }
    
    @Override
    public void execute() {
        Shooter.setShooterRPM(rpm);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
    
}
