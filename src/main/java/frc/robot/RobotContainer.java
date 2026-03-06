// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class RobotContainer {

	// Controllers
	public static final CommandPS5Controller m_driverController = new CommandPS5Controller(0);
	// public static final CommandPS5Controller m_operatorController = new CommandPS5Controller(1);

	// Subsystems

	// Example field point to aim at
	private static final Translation2d kLookAtPoint = new Translation2d(8.27, 4.10);
	private static final double kReducedDriveScale = 0.50;

	// Autonomous
	private SendableChooser<Command> m_autoChooser;

	public RobotContainer() {
		NamedCommands.registerCommand("nothing", Commands.sequence(
		));
		// Autonomous event markers: explicit field-based aiming helpers.

		configureBindings();

		m_autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("AutoR", m_autoChooser);
	}

	private void configureBindings() {
		// Driver Controller
		
	}

	private void setAimOverride(AimOverrideButton button) {
	}

	private void clearAimOverride(AimOverrideButton button) {

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

	public void onAutonomousInit() {
	}

	public void onTeleopInit() {
	}
}
