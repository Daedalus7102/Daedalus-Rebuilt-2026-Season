package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveMode;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class FeedSwerveCommand extends Command {
    private SwerveSubsystem swerveSubsystem;
    private ShooterSubsystem shooterSubsystem;

    public FeedSwerveCommand(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        swerveSubsystem.setMode(DriveMode.AUTO_TEAM);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void end(boolean _i) {
        swerveSubsystem.resetMode();
    }
}