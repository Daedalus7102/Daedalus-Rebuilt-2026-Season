// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.drive.SwerveDrive.SwerveDriveState;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.FeedexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.camera.Vision;
import frc.robot.subsystems.led.LEDController;
import frc.robot.subsystems.led.LEDState;
import frc.robot.tools.AllianceTargetPoses;

public class RobotContainer {
	private enum AimOverrideButton {
		NONE,
		L1,
		L2,
		R1,
		R2
	}

	// Controllers
	public static final CommandPS5Controller m_driverController = new CommandPS5Controller(0);
	public static final CommandPS5Controller m_operatorController = new CommandPS5Controller(1);

	// Subsystems
	private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
	private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
	private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
	private final FeedexerSubsystem m_FeedexerSubsystem = new FeedexerSubsystem();
	private final Vision m_vision = new Vision(m_swerveSubsystem);

	// LED
	private final LEDController m_ledController = new LEDController();
	private boolean m_isIntaking = false;
	private boolean m_isShooting = false;

	// Example field point to aim at
	private static final double kReducedDriveScale = 0.30;
	private AimOverrideButton m_activeAimOverrideButton = AimOverrideButton.NONE;
	private final Timer m_shootIntakeToggleTimer = new Timer();
	private boolean m_shootIntakeOut = true;

	// Autonomous
	private SendableChooser<Command> m_autoChooser;

	public RobotContainer() {
		NamedCommands.registerCommand("nothing", Commands.sequence(
		));
		NamedCommands.registerCommand("IntakeOut", Commands.runOnce(
			() -> m_intakeSubsystem.intakeOut(),
			m_intakeSubsystem
		));
		NamedCommands.registerCommand("IntakeIn", Commands.runOnce(
			() -> m_intakeSubsystem.intakeIn(),
			m_intakeSubsystem
		));
		NamedCommands.registerCommand("RollerOn", Commands.runOnce(
			() -> m_intakeSubsystem.setRoller(1.0),
			m_intakeSubsystem
		));
		NamedCommands.registerCommand("RollerOff", Commands.runOnce(
			() -> m_intakeSubsystem.stopRoller(),
			m_intakeSubsystem
		));

		NamedCommands.registerCommand("AimAndShoot", Commands.sequence(
				m_swerveSubsystem.enableAutoAimAtPoint(AllianceTargetPoses.getTowerTranslationForCurrentAlliance()),
				Commands.run(
						() -> {
							m_ShooterSubsystem.aim(AllianceTargetPoses.getDistanceToTower(m_swerveSubsystem.getPose()));

							if (m_ShooterSubsystem.isReadyToShoot()) {
								m_FeedexerSubsystem.enable();
								m_intakeSubsystem.setPivotManual(-0.3);
								m_intakeSubsystem.setRoller(0.2);
							} else {
								m_FeedexerSubsystem.disable();
								m_intakeSubsystem.stopPivot();
								m_intakeSubsystem.stopRoller();
							}
						},
						m_ShooterSubsystem,
						m_FeedexerSubsystem,
						m_intakeSubsystem
				).withTimeout(15.0)
		).finallyDo((_interrupted) -> {
			m_FeedexerSubsystem.disable();
			m_intakeSubsystem.stopPivot();
			m_intakeSubsystem.stopRoller();
			m_ShooterSubsystem.disable();
			m_swerveSubsystem.disableAutoAimNow();
		}));

		m_swerveSubsystem.setReducedVelocityScale(kReducedDriveScale);

		configureBindings();

		m_autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("AutoR", m_autoChooser);
		SmartDashboard.putNumber("TestAimTargetAngle", 10);
		SmartDashboard.putNumber("TestAimTargetRPM", 4000);
	}

	private void configureBindings() {

		// demos/ninos tontos
		m_operatorController.cross().toggleOnTrue(
				Commands.runOnce(() -> {
					m_swerveSubsystem.setUseReducedVelocity(true);
					SmartDashboard.putBoolean("ReducedSpeedMode", true);
				}, m_swerveSubsystem)
		).toggleOnFalse(
				Commands.runOnce(() -> {
					m_swerveSubsystem.setUseReducedVelocity(false);
					SmartDashboard.putBoolean("ReducedSpeedMode", false);
				}, m_swerveSubsystem)
		);

		Command aimToggleCommand = Commands.run(
				() -> {
					m_ShooterSubsystem.aim(AllianceTargetPoses.getDistanceToTower(m_swerveSubsystem.getPose()));
					updateTowerDistanceDashboard();
				},
				m_ShooterSubsystem
		).finallyDo((_interrupted) -> m_ShooterSubsystem.disable());

		Command shootToggleCommand = Commands.run(
				() -> {
					if (m_ShooterSubsystem.isReadyToShoot()) {
						if (m_shootIntakeToggleTimer.advanceIfElapsed(1)) {
							m_shootIntakeOut = !m_shootIntakeOut;
						}
						if (m_shootIntakeOut) {
							m_intakeSubsystem.intakeOut();
						} else {
							m_intakeSubsystem.intakeIn();
						}
						m_FeedexerSubsystem.enable();
						m_intakeSubsystem.setRoller(0.2);
					} else {
						m_FeedexerSubsystem.disable();
						m_intakeSubsystem.stopRoller();
						m_intakeSubsystem.stopPivot();
					}
				},
				m_FeedexerSubsystem,
				m_intakeSubsystem
		)
		.beforeStarting(() -> {
			m_shootIntakeOut = true;
			m_shootIntakeToggleTimer.restart();
			m_intakeSubsystem.intakeOut();
		})
		.finallyDo((_interrupted) -> {
			m_shootIntakeToggleTimer.stop();
			m_FeedexerSubsystem.disable();
			m_intakeSubsystem.stopRoller();
			m_intakeSubsystem.intakeIn();
		});

		Command aimFieldToggleCommand = Commands.startEnd(
				() -> {
					m_ShooterSubsystem.setHoodAngle(15.0);
					m_ShooterSubsystem.setShooterRPM(6000);
				},
				() -> {
					m_ShooterSubsystem.disable();
					m_FeedexerSubsystem.disable();
				},
				m_ShooterSubsystem
		);

		Command testAimToggleCommand = Commands.startEnd(
				() -> {
					m_ShooterSubsystem.setHoodAngle(SmartDashboard.getNumber("TestAimTargetAngle", 10));
					m_ShooterSubsystem.setShooterRPM(SmartDashboard.getNumber("TestAimTargetRPM", 4000));
				},
				() -> {
					m_ShooterSubsystem.disable();
					m_FeedexerSubsystem.disable();
				},
				m_ShooterSubsystem
		);

		// Driver Controller
		m_driverController.options().onTrue(Commands.runOnce(m_swerveSubsystem::zeroGyro, m_swerveSubsystem));

		m_swerveSubsystem.setJoystickSuppliers(
			() -> applyAllianceTeleopTranslationFlip(-m_driverController.getHID().getLeftY()),
			() -> applyAllianceTeleopTranslationFlip(-m_driverController.getHID().getLeftX()),
			() -> -m_driverController.getHID().getRightX()
		);
		m_swerveSubsystem.setDPadSuppliers(
			() -> dPadXFromPov(m_driverController.getHID().getPOV()),
			() -> dPadYFromPov(m_driverController.getHID().getPOV())
		);

		m_driverController.L1().onTrue(Commands.runOnce(() -> {
			setAimOverride(AimOverrideButton.L1);
			m_intakeSubsystem.intakeOut();
			m_ShooterSubsystem.setHoodAngle(10.0);
		}, m_swerveSubsystem, m_intakeSubsystem));
		m_driverController.L1().onFalse(Commands.runOnce(() -> clearAimOverride(AimOverrideButton.L1), m_swerveSubsystem));

		m_driverController.square().onTrue(Commands.runOnce(this::updateTowerDistanceDashboard, m_swerveSubsystem));

		m_driverController.L2().onTrue(Commands.runOnce(() -> {
			setAimOverride(AimOverrideButton.L2);
			m_intakeSubsystem.intakeOut();
			m_ShooterSubsystem.setHoodAngle(10.0);
		}, m_swerveSubsystem, m_intakeSubsystem));
		m_driverController.L2().onFalse(Commands.runOnce(() -> clearAimOverride(AimOverrideButton.L2), m_swerveSubsystem));

		m_driverController.R1().onTrue(Commands.runOnce(() -> {
			if (m_activeAimOverrideButton == AimOverrideButton.R1) {
				m_swerveSubsystem.setUseReducedVelocity(false);
				clearAimOverride(AimOverrideButton.R1);
			} else {
				m_swerveSubsystem.setUseReducedVelocity(true);
				setAimOverride(AimOverrideButton.R1);
			}
		}, m_swerveSubsystem));
		m_driverController.R1().toggleOnTrue(aimToggleCommand);

		m_driverController.R2().onTrue(Commands.runOnce(() -> {
			if (m_activeAimOverrideButton == AimOverrideButton.R2) {
				m_swerveSubsystem.setUseReducedVelocity(false);
				clearAimOverride(AimOverrideButton.R2);
			} else {
				m_swerveSubsystem.setUseReducedVelocity(true);
				setAimOverride(AimOverrideButton.R2);
			}
		}, m_swerveSubsystem));
		m_driverController.R2().toggleOnTrue(aimFieldToggleCommand);

		// Operator Controller
		m_operatorController.square()
			.toggleOnTrue(Commands.runOnce(() -> m_intakeSubsystem.setRoller(1.0), m_intakeSubsystem))
			.toggleOnFalse(Commands.runOnce(() -> m_intakeSubsystem.stopRoller(), m_intakeSubsystem));

		// L2 — intake with LED tracking
		m_operatorController.L2()
			.whileTrue(Commands.startEnd(
				() -> {
					m_intakeSubsystem.intakeOut();
					m_intakeSubsystem.setRoller(1.0);
					m_isIntaking = true;          // LED flag ON
				},
				() -> {
					m_intakeSubsystem.stopRoller();
					m_isIntaking = false;          // LED flag OFF
				},
				m_intakeSubsystem
			));

		m_operatorController.triangle()
			.toggleOnTrue(Commands.runOnce(() -> m_intakeSubsystem.setPivotPosition(0), m_intakeSubsystem))
			.toggleOnFalse(Commands.runOnce(() -> m_intakeSubsystem.stopPivot(), m_intakeSubsystem));

		m_operatorController.circle()
			.toggleOnTrue(Commands.runOnce(() -> m_intakeSubsystem.setPivotPosition(19), m_intakeSubsystem))
			.toggleOnFalse(Commands.runOnce(() -> m_intakeSubsystem.stopPivot(), m_intakeSubsystem));

		// R1 — shoot with LED tracking
		m_operatorController.R1()
			.onTrue(Commands.runOnce(() -> m_isShooting = true))   // LED flag ON
			.onFalse(Commands.runOnce(() -> m_isShooting = false))  // LED flag OFF
			.whileTrue(shootToggleCommand);
	}

	// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
	// LED UPDATE — called from Robot.java robotPeriodic()
	// Does NOT affect any subsystem logic
	// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
	public void updateLEDs(boolean isAuto, boolean isEndgame) {
		if (isAuto) {
			m_ledController.set(LEDState.AUTO);
			return;
		}
		if (isEndgame) {
			m_ledController.set(LEDState.ENDGAME);
			return;
		}
		if (m_isShooting) {
			double targetRPM = m_ShooterSubsystem.getTargetRPM();
			double currentRPM = m_ShooterSubsystem.getShooterRPM();
			int rpmPercent = targetRPM > 0
				? (int) Math.min(100, (currentRPM / targetRPM) * 100)
				: 0;
			boolean hasTarget = m_vision.hasAprilTagTarget();
			m_ledController.set(LEDState.SHOOT, rpmPercent, hasTarget);
		} else if (m_isIntaking) {
			m_ledController.set(LEDState.INTAKE);
		} else {
			m_ledController.set(LEDState.TELEOP);
		}
	}

	private void setAimOverride(AimOverrideButton button) {
		m_activeAimOverrideButton = button;
		boolean isRedAlliance = AllianceTargetPoses.isCurrentAllianceRed();
		switch (button) {
			case L1 -> {
				m_swerveSubsystem.setUseReducedVelocity(false);
				m_swerveSubsystem.driveFacingAngle(Rotation2d.fromDegrees(isRedAlliance ? 180.0 : 0.0));
			}
			case L2 -> {
				m_swerveSubsystem.setUseReducedVelocity(false);
				m_swerveSubsystem.driveFacingAngle(Rotation2d.fromDegrees(isRedAlliance ? 0.0 : 180.0));
			}
			case R1 -> m_swerveSubsystem.driveFacingPoint(AllianceTargetPoses.getTowerTranslationForCurrentAlliance());
			case R2 -> m_swerveSubsystem.driveFacingPoint(AllianceTargetPoses.getClosestAllianceZoneTranslation(m_swerveSubsystem.getPose()));
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
			m_swerveSubsystem.driveFacingPoint(AllianceTargetPoses.getTowerTranslationForCurrentAlliance());
		} else if (m_activeAimOverrideButton == AimOverrideButton.R2) {
			m_swerveSubsystem.driveFacingPoint(AllianceTargetPoses.getClosestAllianceZoneTranslation(m_swerveSubsystem.getPose()));
		}
	}

	private double dPadXFromPov(int pov) {
		return switch (pov) {
			case 0, 45, 315 -> 1.0;
			case 135, 180, 225 -> -1.0;
			default -> 0.0;
		};
	}

	private double applyAllianceTeleopTranslationFlip(double value) {
		return AllianceTargetPoses.isCurrentAllianceRed() ? -value : value;
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

	private void applyAllianceStartHeading() {
		Rotation2d startHeading = AllianceTargetPoses.isCurrentAllianceRed()
				? Rotation2d.fromDegrees(180.0)
				: new Rotation2d();
		m_swerveSubsystem.setGyroHeading(startHeading);
	}

	public void onAutonomousInit() {
		applyAllianceStartHeading();
		m_swerveSubsystem.disableAutoAimNow();
		m_swerveSubsystem.setUseReducedVelocity(false);
		CommandScheduler.getInstance().schedule(m_swerveSubsystem.setState(SwerveDriveState.AUTO));
	}

	public void onTeleopInit() {
		m_swerveSubsystem.disableAutoAimNow();
		m_swerveSubsystem.syncEstimatorHeadingToGyro();
		m_swerveSubsystem.setUseReducedVelocity(false);
		CommandScheduler.getInstance().schedule(m_swerveSubsystem.setState(SwerveDriveState.IDLE));
	}

	public Runnable dashboardLoop() {
		return () -> {
			SmartDashboard.putBoolean("AllianceIsRed", AllianceTargetPoses.isCurrentAllianceRed());
			SmartDashboard.putBoolean("TeleopTranslationFlippedForRed", AllianceTargetPoses.isCurrentAllianceRed());
			refreshDynamicAimTargets();
			m_swerveSubsystem.updateDashboard();
		};
	}
}