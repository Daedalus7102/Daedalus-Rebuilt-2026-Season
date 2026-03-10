package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class TestAimCommand extends Command{

    private ShooterSubsystem shooterSubsystem;

    public TestAimCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setHoodAngle(SmartDashboard.getNumber("TestAimTargetAngle", 10));
        shooterSubsystem.setShooterRPM(SmartDashboard.getNumber("TestAimTargetRPM", 4000));
    }

    @Override
    public void end(boolean _i) {
        shooterSubsystem.disable();
    }
}