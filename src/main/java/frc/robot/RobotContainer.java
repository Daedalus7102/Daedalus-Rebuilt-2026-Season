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
import frc.robot.commands.drive.ResetGyroComand;
import frc.robot.commands.intake.IntakeAbsorbCommand;
import frc.robot.commands.shooting.ShootCommand;
import frc.robot.commands.shooting.TestAimCommand;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.tools.AllianceTargetPoses;

public class RobotContainer {
	private enum AimOverrideButton {
		NONE,
		L1,
		L2,
		R1
	}

	// Controllers
	public static final CommandPS5Controller m_driverController = new CommandPS5Controller(0);
	public static final CommandPS5Controller m_operatorController = new CommandPS5Controller(1);

	// Subsystems
	private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
	private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
	private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
	private final FeederSubsystem m_FeederSubsystem = new FeederSubsystem();

	// Example field point to aim at
	private static final Translation2d kLookAtPoint = new Translation2d(8.27, 4.10);
	private static final double kReducedDriveScale = 0.30;
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

		m_swerveSubsystem.setReducedVelocityScale(kReducedDriveScale);
		
		configureBindings();

		m_autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("AutoR", m_autoChooser);
		SmartDashboard.putNumber("TestAimTargetAngle", 10);
		SmartDashboard.putNumber("TestAimTargetRPM", 4000);
	}

	private void configureBindings() {

		/* 
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
		*/

		// Driver Controller

		// Zero gyro heading on button press.
		m_driverController.options().onTrue(
			new ResetGyroComand(m_swerveSubsystem));

		m_swerveSubsystem.setJoystickSuppliers(
			() -> -m_driverController.getHID().getLeftY(),
			() -> -m_driverController.getHID().getLeftX(),
			() -> -m_driverController.getHID().getRightX()
		);
		m_swerveSubsystem.setDPadSuppliers(
			() -> dPadXFromPov(m_driverController.getHID().getPOV()),
			() -> dPadYFromPov(m_driverController.getHID().getPOV())
		);

		// Last pressed aim button wins.
		m_driverController.L1().onTrue(Commands.runOnce(() -> {
			setAimOverride(AimOverrideButton.L1);
			m_intakeSubsystem.intakeOut();
		}, m_swerveSubsystem, m_intakeSubsystem));
		m_driverController.L1().onFalse(Commands.runOnce(() -> clearAimOverride(AimOverrideButton.L1), m_swerveSubsystem));

		// Publish current distance to alliance tower and draw line robot->tower on Field2d.
		m_driverController.square().onTrue(Commands.runOnce(this::updateTowerDistanceDashboard, m_swerveSubsystem));

		m_driverController.L2().onTrue(Commands.runOnce(() -> {
			setAimOverride(AimOverrideButton.L2);
			m_intakeSubsystem.intakeOut();
		}, m_swerveSubsystem, m_intakeSubsystem));
		m_driverController.L2().onFalse(Commands.runOnce(() -> clearAimOverride(AimOverrideButton.L2), m_swerveSubsystem));

		// Driver R1: tap once to aim at alliance tower, tap again to cancel.
		m_driverController.R1().onTrue(Commands.runOnce(() -> {
			if (m_activeAimOverrideButton == AimOverrideButton.R1) {
				m_swerveSubsystem.setUseReducedVelocity(false);
				clearAimOverride(AimOverrideButton.R1);
			} else {
				m_swerveSubsystem.setUseReducedVelocity(true);
				setAimOverride(AimOverrideButton.R1);
			}
		}, m_swerveSubsystem));
		m_driverController.R1().toggleOnTrue(new TestAimCommand(m_ShooterSubsystem));


		// Operator Controller
		// Intake test buttons (driver controller)
		m_operatorController.square()
			.toggleOnTrue(Commands.runOnce(() -> m_intakeSubsystem.setRoller(1.0), m_intakeSubsystem))
			.toggleOnFalse(Commands.runOnce(() -> m_intakeSubsystem.stopRoller(), m_intakeSubsystem));

		m_operatorController.L2()
			.whileTrue(new IntakeAbsorbCommand(m_intakeSubsystem));

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
		m_operatorController.R1().whileTrue(new ShootCommand(m_FeederSubsystem, m_ShooterSubsystem, false));

		
	}

	private void setAimOverride(AimOverrideButton button) {
		m_activeAimOverrideButton = button;
		switch (button) {
			case L1 -> {
				m_swerveSubsystem.setUseReducedVelocity(false);
				m_swerveSubsystem.driveFacingAngle(Rotation2d.fromDegrees(0.0));
			}
			case L2 -> {
				m_swerveSubsystem.setUseReducedVelocity(false);
				m_swerveSubsystem.driveFacingAngle(Rotation2d.fromDegrees(180.0));
			}
			case R1 -> m_swerveSubsystem.driveFacingPoint(AllianceTargetPoses.getTowerTranslationForCurrentAlliance());
			case NONE -> {
				m_swerveSubsystem.setUseFixedOmega(false);
				m_swerveSubsystem.setUseReducedVelocity(false);
				return;
			}
		}
		m_swerveSubsystem.setUseFixedOmega(true);
	}

	private void clearAimOverride(AimOverrideButton button) {
		if (m_activeAimOverrideButton == button) {
			m_activeAimOverrideButton = AimOverrideButton.NONE;
			m_swerveSubsystem.setUseFixedOmega(false);
			m_swerveSubsystem.setUseReducedVelocity(false);
		}
	}

	private void updateTowerDistanceDashboard() {
		Translation2d robotTranslation = m_swerveSubsystem.getPose().getTranslation();
		Translation2d towerTranslation = AllianceTargetPoses.getTowerTranslationForCurrentAlliance();
		double distanceMeters = AllianceTargetPoses.getDistanceToTower(m_swerveSubsystem.getPose());

		SmartDashboard.putNumber("AllianceTowerDistanceM", distanceMeters);
		m_swerveSubsystem.setFieldLine("AllianceTowerLine", robotTranslation, towerTranslation);
	}

	private void refreshDynamicAimTargets() {
		if (m_activeAimOverrideButton == AimOverrideButton.R1) {
			// Keep refreshing tower target so heading keeps updating with robot pose.
			m_swerveSubsystem.driveFacingPoint(AllianceTargetPoses.getTowerTranslationForCurrentAlliance());
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
		m_swerveSubsystem.setUseReducedVelocity(false);
		CommandScheduler.getInstance().schedule(m_swerveSubsystem.setState(SwerveDriveState.AUTO));
	}

	public void onTeleopInit() {
		m_swerveSubsystem.disableAutoAimNow();
		m_swerveSubsystem.setUseReducedVelocity(false);
		CommandScheduler.getInstance().schedule(m_swerveSubsystem.setState(SwerveDriveState.IDLE));
	}

	public Runnable dashboardLoop() {
		return () -> {
			refreshDynamicAimTargets();
			m_swerveSubsystem.updateDashboard();
		};
	}
}
