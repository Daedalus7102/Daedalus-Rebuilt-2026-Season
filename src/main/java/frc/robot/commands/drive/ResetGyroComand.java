package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class ResetGyroComand extends Command {
    private SwerveSubsystem swerveSubsystem;

    public ResetGyroComand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void initialize() {
        swerveSubsystem.zeroGyro();
        addRequirements(swerveSubsystem);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
