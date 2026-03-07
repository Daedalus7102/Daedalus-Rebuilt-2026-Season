package frc.robot.subsystems.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class SpoolShooter extends Command {
    private final ShooterSubsystem shooter;
    
    private final DoubleSupplier distance; // para poder cambiar el distance mientras se ejecuta el comando
    
    public SpoolShooter(ShooterSubsystem shooter, DoubleSupplier distance) {
        this.distance = distance;
        this.shooter = shooter;
        addRequirements(shooter);
    }
    
    @Override
    public void execute() {
        shooter.aim(distance.getAsDouble());
        //shooter.setMeasuredRPM(distance.getAsDouble());
    }

    @Override
    public void end(boolean _i) {
        shooter.disable();
    }
}
