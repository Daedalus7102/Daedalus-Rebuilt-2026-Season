// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveMode;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.tools.AllianceTargetPoses;

public class RobotContainer {

	// Controllers
	public static final CommandPS5Controller m_driverController = new CommandPS5Controller(0);
	public static final CommandPS5Controller m_operatorController = new CommandPS5Controller(1);

	// Subsystems
	private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(
			() -> m_driverController.getHID().getLeftX(),
			() -> m_driverController.getHID().getLeftY(),
			() -> m_driverController.getHID().getRightX(),
			() -> dPadXFromPov(m_driverController.getHID().getPOV()),
			() -> dPadYFromPov(m_driverController.getHID().getPOV())
	);
	private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
	private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
	private final FeederSubsystem m_FeederSubsystem = new FeederSubsystem();

	// Example field point to aim at
	private static final Translation2d kLookAtPoint = new Translation2d(4.62, 4.03);
	private static final double kReducedDriveScale = 0.50;

	// Autonomous
	private SendableChooser<Command> m_autoChooser;

	public RobotContainer() {
		NamedCommands.registerCommand("nothing", Commands.sequence(
		));
		// Autonomous event markers: explicit field-based aiming helpers.
		NamedCommands.registerCommand("AimHubOn", Commands.runOnce(() -> m_swerveSubsystem.setMode(DriveMode.AUTO_HUB), m_swerveSubsystem));
		NamedCommands.registerCommand("AimHubOff", Commands.runOnce(m_swerveSubsystem::resetMode, m_swerveSubsystem));
		NamedCommands.registerCommand("AimTeamOn", Commands.runOnce(() -> m_swerveSubsystem.setMode(DriveMode.AUTO_TEAM), m_swerveSubsystem));
		NamedCommands.registerCommand("AimTeamOff", Commands.runOnce(m_swerveSubsystem::resetMode, m_swerveSubsystem));

		configureBindings();
		m_swerveSubsystem.setHubPos(kLookAtPoint);

		m_autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("AutoR", m_autoChooser);
		SmartDashboard.putNumber("TestAimTargetAngle", 10);
		SmartDashboard.putNumber("TestAimTargetRPM", 4000);
	}

	private void configureBindings() {

		Command aimToggleCommand = Commands.run(
				() -> m_ShooterSubsystem.aim(10),
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

		Command testAimToggleCommand = Commands.startEnd(
				() -> {
					m_ShooterSubsystem.setHoodAngle(SmartDashboard.getNumber("TestAimTargetAngle", 10));
					m_ShooterSubsystem.setShooterRPM(SmartDashboard.getNumber("TestAimTargetRPM", 4000));
				},
				() -> {
					m_ShooterSubsystem.disable();
					m_FeederSubsystem.disable();
				},
				m_ShooterSubsystem
		);
		// Driver Controller
		m_driverController.square().onTrue(Commands.runOnce(this::updateTowerDistanceDashboard, m_swerveSubsystem));

		m_driverController.L1().onTrue(Commands.runOnce(() -> m_swerveSubsystem.setMode(DriveMode.AUTO_TEAM), m_swerveSubsystem));
		m_driverController.L1().onFalse(Commands.runOnce(m_swerveSubsystem::resetMode, m_swerveSubsystem));

		m_driverController.R1().onTrue(Commands.runOnce(() -> m_swerveSubsystem.setMode(DriveMode.AUTO_HUB), m_swerveSubsystem));
		m_driverController.R1().onFalse(Commands.runOnce(m_swerveSubsystem::resetMode, m_swerveSubsystem));

		m_driverController.options().onTrue(Commands.runOnce(m_swerveSubsystem::resetOdometryRotation, m_swerveSubsystem));

		// Operator Controller
		// Intake test buttons (driver controller)
		m_operatorController.square()
			.toggleOnTrue(Commands.runOnce(() -> m_intakeSubsystem.setRoller(1.0), m_intakeSubsystem))
			.toggleOnFalse(Commands.runOnce(() -> m_intakeSubsystem.stopRoller(), m_intakeSubsystem));

		m_operatorController.L2()
			.whileTrue(Commands.startEnd(
				() -> {
					m_intakeSubsystem.intakeOut();
					m_intakeSubsystem.setRoller(1.0);
				},
				() -> m_intakeSubsystem.stopRoller(),
				m_intakeSubsystem
			));

		m_operatorController.triangle()
			// .toggleOnTrue(Commands.runOnce(() -> m_intakeSubsystem.setPivotManual(-0.8), m_intakeSubsystem))
			.toggleOnTrue(Commands.runOnce(() -> m_intakeSubsystem.setPivotPosition(0), m_intakeSubsystem))
			.toggleOnFalse(Commands.runOnce(() -> m_intakeSubsystem.stopPivot(), m_intakeSubsystem));

		m_operatorController.circle()
			// .toggleOnTrue(Commands.runOnce(() -> m_intakeSubsystem.setPivotManual(0.8), m_intakeSubsystem))
			.toggleOnTrue(Commands.runOnce(() -> m_intakeSubsystem.setPivotPosition(19), m_intakeSubsystem))
			.toggleOnFalse(Commands.runOnce(() -> m_intakeSubsystem.stopPivot(), m_intakeSubsystem));

		// Operator L2: press once to start aim, press again to stop aim.
		// m_operatorController.L1().toggleOnTrue(aimToggleCommand);

		// Operator R1: shoot only while held.
		m_operatorController.R1().whileTrue(shootToggleCommand);


	}

	private void updateTowerDistanceDashboard() {
		Translation2d robotTranslation = m_swerveSubsystem.swerveDrive.getPose().getTranslation();
		Translation2d towerTranslation = AllianceTargetPoses.getTowerTranslationForCurrentAlliance();
		double distanceMeters = AllianceTargetPoses.getDistanceToTower(m_swerveSubsystem.swerveDrive.getPose());

		SmartDashboard.putNumber("AllianceTowerDistanceM", distanceMeters);
		m_swerveSubsystem.setFieldLine("AllianceTowerLine", robotTranslation, towerTranslation);
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
		m_swerveSubsystem.resetMode();
		m_swerveSubsystem.setScaleInput(false);
		//CommandScheduler.getInstance().schedule(m_swerveSubsystem.setState(SwerveDriveState.AUTO));
	}

	public void onTeleopInit() {
		m_swerveSubsystem.resetMode();
		m_swerveSubsystem.setScaleInput(false);
		//CommandScheduler.getInstance().schedule(m_swerveSubsystem.setState(SwerveDriveState.IDLE));
	}

	public Runnable dashboardLoop() {
		return () -> {
		};
	}
}
