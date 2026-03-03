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
import frc.robot.subsystems.drive.SwerveDrive.SwerveDriveState;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class RobotContainer {
	private enum AimOverrideButton {
		NONE,
		L1,
		L2,
		R1
	}

	// Controllers
	public static final CommandPS5Controller m_driverController = new CommandPS5Controller(0);
	// public static final CommandPS5Controller m_operatorController = new CommandPS5Controller(1);

	// Subsystems
	private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

	// Example field point to aim at
	private static final Translation2d kLookAtPoint = new Translation2d(8.27, 4.10);
	private AimOverrideButton m_activeAimOverrideButton = AimOverrideButton.NONE;

	// Autonomous
	private SendableChooser<Command> m_autoChooser;

	public RobotContainer() {
		NamedCommands.registerCommand("nothing", Commands.sequence(
		));
		// Autonomous event markers: explicit field-based aiming helpers.
		NamedCommands.registerCommand("AimSpeakerOn", m_swerveSubsystem.enableAutoAimAtPoint(kLookAtPoint));
		NamedCommands.registerCommand("AimSpeakerOff", m_swerveSubsystem.disableAutoAim());
		NamedCommands.registerCommand("AimForwardOn", m_swerveSubsystem.enableAutoAimAtAngle(Rotation2d.fromDegrees(0.0)));
		NamedCommands.registerCommand("AimForwardOff", m_swerveSubsystem.disableAutoAim());
		
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

		// Teleop convenience wrappers (driver intent first):
		// Last pressed aim button wins.
		m_driverController.L1().onTrue(Commands.runOnce(() -> setAimOverride(AimOverrideButton.L1), m_swerveSubsystem));
		m_driverController.L1().onFalse(Commands.runOnce(() -> clearAimOverride(AimOverrideButton.L1), m_swerveSubsystem));

		m_driverController.L2().onTrue(Commands.runOnce(() -> setAimOverride(AimOverrideButton.L2), m_swerveSubsystem));
		m_driverController.L2().onFalse(Commands.runOnce(() -> clearAimOverride(AimOverrideButton.L2), m_swerveSubsystem));

		m_driverController.R1().onTrue(Commands.runOnce(() -> setAimOverride(AimOverrideButton.R1), m_swerveSubsystem));
		m_driverController.R1().onFalse(Commands.runOnce(() -> clearAimOverride(AimOverrideButton.R1), m_swerveSubsystem));

		// Zero gyro heading on button press.
		m_driverController.options().onTrue(Commands.runOnce(m_swerveSubsystem::zeroGyro, m_swerveSubsystem));

		// Operator Controller
		
	}

	private void setAimOverride(AimOverrideButton button) {
		m_activeAimOverrideButton = button;
		switch (button) {
			case L1 -> m_swerveSubsystem.driveFacingAngle(Rotation2d.fromDegrees(0.0));
			case L2 -> m_swerveSubsystem.driveFacingAngle(Rotation2d.fromDegrees(180.0));
			case R1 -> m_swerveSubsystem.driveFacingPoint(kLookAtPoint);
			case NONE -> {
				m_swerveSubsystem.setUseFixedOmega(false);
				return;
			}
		}
		m_swerveSubsystem.setUseFixedOmega(true);
	}

	private void clearAimOverride(AimOverrideButton button) {
		if (m_activeAimOverrideButton == button) {
			m_activeAimOverrideButton = AimOverrideButton.NONE;
			m_swerveSubsystem.setUseFixedOmega(false);
		}
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
		m_swerveSubsystem.disableAutoAimNow();
		CommandScheduler.getInstance().schedule(m_swerveSubsystem.setState(SwerveDriveState.AUTO));
	}

	public void onTeleopInit() {
		m_swerveSubsystem.disableAutoAimNow();
		CommandScheduler.getInstance().schedule(m_swerveSubsystem.setState(SwerveDriveState.IDLE));
	}

	public Runnable dashboardLoop() {
		return () -> {
			m_swerveSubsystem.updateDashboard();
		};
	}
}
