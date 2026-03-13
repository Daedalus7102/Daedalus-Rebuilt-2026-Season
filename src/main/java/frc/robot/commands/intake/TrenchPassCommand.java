package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class TrenchPassCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    public TrenchPassCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(intakeSubsystem, shooterSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.intakeOut();
        shooterSubsystem.setHoodAngle(ShooterConstants.trenchAngle);
    }

    @Override
    public void end(boolean _interrupted) {
        shooterSubsystem.setHoodAngle(10);
        intakeSubsystem.stopPivot();
    }
}
