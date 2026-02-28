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
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RobotContainer {

	// Controllers
	public static final CommandPS5Controller m_driverController = new CommandPS5Controller(0);
	// public static final CommandPS5Controller m_operatorController = new CommandPS5Controller(1);

	// Subsystems
	private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
	private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

	// Autonomous
	private SendableChooser<Command> m_autoChooser;

	public RobotContainer() {
		NamedCommands.registerCommand("nothing", Commands.sequence(
		));
		
		configureBindings();

		m_autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("AutoR", m_autoChooser);
	}

	private void configureBindings() {
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

		// Operator Controller
		// Intake test buttons (driver controller)
		m_driverController.square()
			.toggleOnTrue(Commands.runOnce(() -> m_intakeSubsystem.setRoller(1.0), m_intakeSubsystem))
			.toggleOnFalse(Commands.runOnce(() -> m_intakeSubsystem.stopRoller(), m_intakeSubsystem));

		m_driverController.cross()
			.onTrue(Commands.runOnce(() -> m_intakeSubsystem.stopRoller(), m_intakeSubsystem));

		m_driverController.triangle()
			.toggleOnTrue(Commands.runOnce(() -> m_intakeSubsystem.setPivotManual(-0.4), m_intakeSubsystem))
			.toggleOnFalse(Commands.runOnce(() -> m_intakeSubsystem.stopPivot(), m_intakeSubsystem));

		m_driverController.circle()
			.toggleOnTrue(Commands.runOnce(() -> m_intakeSubsystem.setPivotManual(1.0), m_intakeSubsystem))
			.toggleOnFalse(Commands.runOnce(() -> m_intakeSubsystem.stopPivot(), m_intakeSubsystem));

		// Zero intake pivot relative encoder at current physical "in" position
		m_driverController.options()
			.onTrue(Commands.runOnce(m_intakeSubsystem::zeroPivotAtInPosition, m_intakeSubsystem));
		
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
			m_intakeSubsystem.updateDashboard();
		};
	}
}
