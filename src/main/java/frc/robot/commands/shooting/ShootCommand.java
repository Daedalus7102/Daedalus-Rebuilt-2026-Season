package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootCommand extends Command {

    private FeederSubsystem feederSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private boolean bypassBlock;

    public ShootCommand(FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem, boolean bypassBlock) {
        this.feederSubsystem = feederSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.bypassBlock = bypassBlock;
        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        if (bypassBlock) {
            feederSubsystem.enable();
        }
    }

    @Override
    public void execute() {
        if (bypassBlock) {
            return;
        }

        if (shooterSubsystem.isReadyToShoot()) {
            feederSubsystem.enable();
            return;
        }

        feederSubsystem.disable();
    }

    @Override
    public void end(boolean _i) {
        feederSubsystem.disable();
    }
}
