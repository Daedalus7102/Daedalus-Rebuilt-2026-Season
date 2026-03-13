package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveMode;

public class AimSwerveCommand extends Command {
    private SwerveSubsystem swerveSubsystem;

    public AimSwerveCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void initialize() {
        swerveSubsystem.setMode(DriveMode.AUTO_HUB);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void end(boolean _i) {
        swerveSubsystem.resetMode();
    }
}