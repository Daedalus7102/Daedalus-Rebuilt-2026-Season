package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class SpoolShooterCommand extends Command {
    private final ShooterSubsystem shooter;
    
    private final DoubleSupplier distance; // para poder cambiar el distance mientras se ejecuta el comando
    
    public SpoolShooterCommand(ShooterSubsystem shooter, DoubleSupplier distance) {
        this.distance = distance;
        this.shooter = shooter;
        addRequirements(shooter);
    }
    
    @Override
    public void execute() {
        shooter.aim(distance.getAsDouble());
        shooter.setMeasuredRPM(distance.getAsDouble());
    }

    @Override
    public void end(boolean _i) {
        shooter.disable();
    }
}
