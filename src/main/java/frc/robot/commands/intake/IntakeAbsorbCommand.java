package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeAbsorbCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;

    public IntakeAbsorbCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.intakeOut();
        intakeSubsystem.setRoller(1.0);
    }

    @Override
    public void end(boolean _i) {
        intakeSubsystem.stopRoller();
    }
}
