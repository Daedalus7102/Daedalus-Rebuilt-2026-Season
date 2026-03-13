// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.drive.AimSwerveCommand;
import frc.robot.commands.drive.FeedSwerveCommand;
import frc.robot.commands.intake.IntakeAbsorbCommand;
import frc.robot.commands.intake.TrenchPassCommand;
import frc.robot.commands.shooting.ActivateFeederCommand;
import frc.robot.commands.shooting.SpoolShooterCommand;
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
		NamedCommands.registerCommand("nothing", Commands.sequence());
                
		// Intake
		NamedCommands.registerCommand("DeployIntake",
			Commands.runOnce(m_intakeSubsystem::intakeOut, m_intakeSubsystem));
		NamedCommands.registerCommand("StartRollers",
			Commands.runOnce(() -> m_intakeSubsystem.setRoller(1.0), m_intakeSubsystem));
		NamedCommands.registerCommand("StopRollers",
			Commands.runOnce(m_intakeSubsystem::stopRoller, m_intakeSubsystem));
		NamedCommands.registerCommand("RetractIntake",
			Commands.runOnce(m_intakeSubsystem::intakeIn, m_intakeSubsystem));

		// Autonomous event markers: explicit field-based aiming helpers.
		NamedCommands.registerCommand("AimHubOn", Commands.runOnce(() -> m_swerveSubsystem.setMode(DriveMode.AUTO_HUB), m_swerveSubsystem));
		NamedCommands.registerCommand("AimHubOff", Commands.runOnce(m_swerveSubsystem::resetMode, m_swerveSubsystem));
		NamedCommands.registerCommand("AimTeamOn", Commands.runOnce(() -> m_swerveSubsystem.setMode(DriveMode.AUTO_TEAM), m_swerveSubsystem));
		NamedCommands.registerCommand("AimTeamOff", Commands.runOnce(m_swerveSubsystem::resetMode, m_swerveSubsystem));

		configureBindings();
		m_swerveSubsystem.setHubPos(kLookAtPoint);

		m_autoChooser = AutoBuilder.buildAutoChooser();
		m_autoChooser.addOption("Blue Left 2 Cycle",  blueLeft2Cycle());
		m_autoChooser.addOption("Blue Right 2 Cycle", blueRight2Cycle());
		m_autoChooser.addOption("Blue Middle Go L",   singlePath("BlueMiddle_Go_L"));
		m_autoChooser.addOption("Blue Middle Go R",   singlePath("BlueMiddle_Go_R"));

		SmartDashboard.putData("AutoR", m_autoChooser);
		SmartDashboard.putNumber("TestAimTargetAngle", 10);
		SmartDashboard.putNumber("TestAimTargetRPM", 4000);
                
		m_swerveSubsystem.inputMultiplier = kReducedDriveScale;
	}

	private void configureBindings() {
                /* ---- Driver Controller ---- */

		// Zero gyro heading on button press.
		// m_driverController.options().onTrue(new ResetGyroComand(m_swerveSubsystem));

		// Publish current distance to alliance tower and draw line robot->tower on Field2d.
		// m_driverController.square().onTrue(Commands.runOnce(this::updateTowerDistanceDashboard, m_swerveSubsystem));

		// Lower the intake for when you're on the trench
		m_driverController.cross().whileTrue(new TrenchPassCommand(m_intakeSubsystem, m_ShooterSubsystem));

		// For feeding (Hold to enable)
		m_driverController.L2().whileTrue(new FeedSwerveCommand(m_swerveSubsystem, m_ShooterSubsystem));
                
		// For shooting (Hold to enable)
		m_driverController.R2().whileTrue(new AimSwerveCommand(m_swerveSubsystem, m_ShooterSubsystem));


		/* ---- Operator Controller ---- */

		// Intake test buttons (driver controller)
		m_operatorController.square()
			.toggleOnTrue(Commands.runOnce(() -> m_intakeSubsystem.setRoller(1.0), m_intakeSubsystem))
			.toggleOnFalse(Commands.runOnce(() -> m_intakeSubsystem.stopRoller(), m_intakeSubsystem));

		m_operatorController.L2().whileTrue(new IntakeAbsorbCommand(m_intakeSubsystem));

		m_operatorController.L1()
			// .toggleOnTrue(Commands.runOnce(() -> m_intakeSubsystem.setPivotManual(-0.8), m_intakeSubsystem))
			.toggleOnTrue(Commands.runOnce(() -> m_intakeSubsystem.intakeIn(), m_intakeSubsystem))
			.toggleOnFalse(Commands.runOnce(() -> m_intakeSubsystem.stopPivot(), m_intakeSubsystem));
                        
		// Operator L2: press once to start aim, press again to stop aim.
		// m_operatorController.L1().toggleOnTrue(aimToggleCommand);

		// Operator R1: shoot only while held.
		m_operatorController.cross().whileTrue(new ActivateFeederCommand(m_FeederSubsystem, m_ShooterSubsystem, true));
		m_operatorController.R2().whileTrue(
				new SpoolShooterCommand(m_ShooterSubsystem, () -> AllianceTargetPoses.getDistanceToTower(m_swerveSubsystem.swerveDrive.getPose())));
                m_operatorController.R2().onFalse(new InstantCommand(() -> m_ShooterSubsystem.disable()));
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

	private Command singlePath(String name) {
		try {
			return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(name));
		} catch (Exception e) {
			DriverStation.reportError(
				"Failed to load path: " + name + " - " + e.getMessage(),
				e.getStackTrace());
			return Commands.none();
		}
	}

	/**
	 * Blue Left 2-Cycle: Collect -> Return+Shoot -> Collect -> Return+Shoot
	 * BlueLeft_Return ends at BlueLeft_Collect start pose.
	 */
	private Command blueLeft2Cycle() {
		return Commands.sequence(
			singlePath("BlueLeft_Collect"),
			singlePath("BlueLeft_Return"),
			singlePath("BlueLeft_Collect"),
			singlePath("BlueLeft_Return")
		);
	}

	/**
	 * Blue Right 2-Cycle: Collect -> Return+Shoot -> Collect -> Return+Shoot
	 * BlueRight_Return ends at BlueRight_Collect start pose.
	 */
	private Command blueRight2Cycle() {
		return Commands.sequence(
			singlePath("BlueRight_Collect"),
			singlePath("BlueRight_Return"),
			singlePath("BlueRight_Collect"),
			singlePath("BlueRight_Return")
		);
	}

	/**
	 * Blue Middle to Left Cycle:
	 * Middle->Left collect -> Left return+shoot -> Left collect -> Left return+shoot
	 */
	private Command blueMiddleToLeftCycle() {
		return Commands.sequence(
			singlePath("BlueMiddle_Go_L"),
			singlePath("BlueLeft_Return"),
			singlePath("BlueLeft_Collect"),
			singlePath("BlueLeft_Return")
		);
	}

	/**
	 * Blue Middle to Right Cycle:
	 * Middle->Right collect -> Right return+shoot -> Right collect -> Right return+shoot
	 */
	private Command blueMiddleToRightCycle() {
		return Commands.sequence(
			singlePath("BlueMiddle_Go_R"),
			singlePath("BlueRight_Return"),
			singlePath("BlueRight_Collect"),
			singlePath("BlueRight_Return")
		);
	}

	public Command getAutonomousCommand() {
		return m_autoChooser.getSelected();
	}


	public void onAutonomousInit() {
		m_swerveSubsystem.resetMode();
		m_swerveSubsystem.setScaleInput(false);
		// leds.set(LEDState.AUTO);
		// CommandScheduler.getInstance().schedule(m_swerveSubsystem.setState(SwerveDriveState.AUTO));
	}

	public void onTeleopInit() {
		m_swerveSubsystem.resetMode();
		m_swerveSubsystem.setScaleInput(false);
		//leds.set(LEDState.OFF);
		//CommandScheduler.getInstance().schedule(m_swerveSubsystem.setState(SwerveDriveState.IDLE));
	}

	public Runnable dashboardLoop() {
		return () -> {
		};
	}
}