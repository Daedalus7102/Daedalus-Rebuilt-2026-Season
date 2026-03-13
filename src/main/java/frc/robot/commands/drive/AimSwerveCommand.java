package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveMode;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.tools.AllianceTargetPoses;

public class AimSwerveCommand extends Command {
    private SwerveSubsystem swerveSubsystem;
    private ShooterSubsystem shooterSubsystem;

    public AimSwerveCommand(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        swerveSubsystem.setMode(DriveMode.AUTO_HUB);
        shooterSubsystem.aim(AllianceTargetPoses.getDistanceToTower(swerveSubsystem.swerveDrive.getPose()));
        addRequirements(swerveSubsystem);
    }

    @Override
    public void end(boolean _i) {
        shooterSubsystem.setHoodAngle(10);
        swerveSubsystem.resetMode();
    }
}