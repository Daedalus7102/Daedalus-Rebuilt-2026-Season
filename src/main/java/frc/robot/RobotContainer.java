// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.shooter.FeederSubsystem;
import frc.robot.subsystems.shooter.LookUpTable;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {

	// Controllers
	public static final CommandPS5Controller m_driverController = new CommandPS5Controller(0);
	public static final CommandPS5Controller m_operatorController = new CommandPS5Controller(1);

	// Subsystems
	private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
	private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
	private final FeederSubsystem m_FeederSubsystem = new FeederSubsystem();

	// Autonomous
	private SendableChooser<Command> m_autoChooser;

	public RobotContainer() {
		NamedCommands.registerCommand("nothing", Commands.sequence(
		));
		SmartDashboard.putNumber("HoodSetAngleDeg", Constants.ShooterConstants.minHoodAngle);
		SmartDashboard.putNumber("ShooterDistanceM", 3.0);
		SmartDashboard.putNumber("TestAimDistanceM", 0.0);
		SmartDashboard.putNumber("TestAimTargetAngle", 0.0);
		SmartDashboard.putNumber("TestAimTargetRPM", 0.0);

		configureBindings();

		m_autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("AutoR", m_autoChooser);
	}

	private void configureBindings() {
		Command aimToggleCommand = Commands.run(
				() -> m_ShooterSubsystem.aim(getShooterDistanceMeters()),
				m_ShooterSubsystem
		).finallyDo((_interrupted) -> m_ShooterSubsystem.disable());

		Command shootToggleCommand = Commands.runEnd(
				() -> {
					if (m_ShooterSubsystem.isReadyToShoot()) {
						m_FeederSubsystem.enable();
					} else {
						m_FeederSubsystem.disable();
					}
				},
				m_FeederSubsystem::disable,
				m_FeederSubsystem
		);

		// Driver Controller
		m_swerveSubsystem.setJoystickSuppliers(
				() -> -m_driverController.getHID().getLeftY(),
				() -> -m_driverController.getHID().getLeftX(),
				() -> -m_driverController.getHID().getRightX()
		);
		m_swerveSubsystem.setDPadSuppliers(
				() -> dPadXFromPov(m_driverController.getHID().getPOV()),
				() -> dPadYFromPov(m_driverController.getHID().getPOV())
		);

		// Operator L2: press once to start aim, press again to stop aim.
		m_operatorController.L2().toggleOnTrue(aimToggleCommand);

		// Operator R2: press once to start shooting, press again to stop shooting.
		m_operatorController.R2().toggleOnTrue(shootToggleCommand);

		// Test: press square once to set hood angle and shooter RPM from LUT point.
		m_operatorController.square().onTrue(Commands.runOnce(
				() -> {
					m_ShooterSubsystem.setHoodAngle(SmartDashboard.getNumber("TestAimTargetAngle", 10));
					m_ShooterSubsystem.setShooterRPM(SmartDashboard.getNumber("TestAimTargetRPM", 5000));
				},
				m_ShooterSubsystem
		));

	}

	private double getShooterDistanceMeters() {
		return SmartDashboard.getNumber("ShooterDistanceM", 3.0);
	}

	private double dPadXFromPov(int pov) {
		return switch (pov) {
			case 0, 45, 315 -> 1.0;
			case 135, 180, 225 -> -1.0;
			default -> 0.0;
		};
	}

	private double dPadYFromPov(int pov) {
		return switch (pov) {
			case 315, 270, 225 -> 1.0;
			case 45, 90, 135 -> -1.0;
			default -> 0.0;
		};
	}

	public Command getAutonomousCommand() {
		return m_autoChooser.getSelected();
	}

	public Runnable dashboardLoop() {
		return () -> {
			m_swerveSubsystem.updateDashboard();
		};
	}
}
