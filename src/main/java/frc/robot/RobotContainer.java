// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.ShooterConstants;
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

    // Field point to aim at
    private static final Translation2d kLookAtPoint = new Translation2d(4.62, 4.03);

    // ── Choreo ──────────────────────────────────────────────────────────────────
    // AutoFactory is created here (robot scope), NOT inside the drive subsystem.
    // It wires together: pose supplier, odometry reset, trajectory follower,
    // alliance-flip flag, and the drive subsystem requirement.
    private final AutoFactory m_autoFactory;

    // AutoChooser lazily builds routines — nothing is loaded until selected,
    // and nothing runs until the DS enables autonomous.
    private final AutoChooser m_autoChooser;

    public RobotContainer() {
        // ── AutoFactory setup ────────────────────────────────────────────────
        m_autoFactory = new AutoFactory(
                m_swerveSubsystem::getPose,          // () -> Pose2d
                m_swerveSubsystem::resetOdometry,    // Pose2d -> void
                m_swerveSubsystem::followTrajectory, // SwerveSample -> void  (see SwerveSubsystem)
                true,                                // flip trajectories for Red alliance
                m_swerveSubsystem                    // drive subsystem requirement
        );

        // Global event bindings — these fire on every trajectory that has a
        // matching event marker name, across ALL routines.
        // (Per-trajectory overrides are done inside each routine method below.)
        m_autoFactory
                .bind("DeployIntake",
                        Commands.runOnce(m_intakeSubsystem::intakeOut, m_intakeSubsystem))
                .bind("StartRollers",
                        Commands.runOnce(() -> m_intakeSubsystem.setRoller(1.0), m_intakeSubsystem))
                .bind("StopRollers",
                        Commands.runOnce(m_intakeSubsystem::stopRoller, m_intakeSubsystem))
                .bind("RetractIntake",
                        Commands.runOnce(m_intakeSubsystem::intakeIn, m_intakeSubsystem))
                .bind("SpinUpShooter",
                        Commands.runOnce(
                                () -> m_ShooterSubsystem.setShooterRPM(ShooterConstants.shooterTargetRPM),
                                m_ShooterSubsystem))
                .bind("AimAndShoot", Commands.sequence(
                        Commands.runOnce(
                                () -> m_ShooterSubsystem.setHoodAngle(ShooterConstants.feedingHoodAngle),
                                m_ShooterSubsystem),
                        Commands.waitUntil(m_ShooterSubsystem::isReadyToShoot),
                        Commands.runOnce(m_FeederSubsystem::enable, m_FeederSubsystem),
                        Commands.waitSeconds(0.5),
                        Commands.runOnce(() -> {
                            m_FeederSubsystem.disable();
                            m_ShooterSubsystem.disable();
                        }, m_ShooterSubsystem, m_FeederSubsystem)
                ))
                .bind("AimHubOn",
                        Commands.runOnce(() -> m_swerveSubsystem.setMode(DriveMode.AUTO_HUB), m_swerveSubsystem))
                .bind("AimHubOff",
                        Commands.runOnce(m_swerveSubsystem::resetMode, m_swerveSubsystem))
                .bind("AimTeamOn",
                        Commands.runOnce(() -> m_swerveSubsystem.setMode(DriveMode.AUTO_TEAM), m_swerveSubsystem))
                .bind("AimTeamOff",
                        Commands.runOnce(m_swerveSubsystem::resetMode, m_swerveSubsystem));

        //AutoChooser setup
        // addRoutine() accepts a supplier; the routine is only built when selected.
        // addCmd() does the same for plain Command-based autos.
        m_autoChooser = new AutoChooser();
        m_autoChooser.addRoutine("Blue Left 2 Cycle",   this::blueLeft2Cycle);
        m_autoChooser.addRoutine("Blue Right 2 Cycle",  this::blueRight2Cycle);
        m_autoChooser.addRoutine("Blue Middle Go L",    this::blueMiddleToLeftCycle);
        m_autoChooser.addRoutine("Blue Middle Go R",    this::blueMiddleToRightCycle);

        // Publish chooser to SmartDashboard (same key as before)
        SmartDashboard.putData("AutoR", m_autoChooser);

        // selectedCommandScheduler() returns a command that runs whichever
        // routine/command is currently selected. Binding it to the autonomous
        // mode trigger means it starts when the DS enables autonomous and
        // stops when autonomous ends — no manual scheduling needed in Robot.java.
        RobotModeTriggers.autonomous().whileTrue(m_autoChooser.selectedCommandScheduler());

        // ── Misc dashboard ───────────────────────────────────────────────────
        SmartDashboard.putNumber("TestAimTargetAngle", 10);
        SmartDashboard.putNumber("TestAimTargetRPM", 4000);

        m_swerveSubsystem.setHubPos(kLookAtPoint);

        configureBindings();
    }

    //Button bindings
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

        // Driver
        m_driverController.square().onTrue(
                Commands.runOnce(this::updateTowerDistanceDashboard, m_swerveSubsystem));

        m_driverController.L1().onTrue(
                Commands.runOnce(() -> m_swerveSubsystem.setMode(DriveMode.AUTO_TEAM), m_swerveSubsystem));
        m_driverController.L1().onFalse(
                Commands.runOnce(m_swerveSubsystem::resetMode, m_swerveSubsystem));

        m_driverController.R1().onTrue(
                Commands.runOnce(() -> m_swerveSubsystem.setMode(DriveMode.AUTO_HUB), m_swerveSubsystem));
        m_driverController.R1().onFalse(
                Commands.runOnce(m_swerveSubsystem::resetMode, m_swerveSubsystem));

        m_driverController.options().onTrue(
                Commands.runOnce(m_swerveSubsystem::resetOdometryRotation, m_swerveSubsystem));

        // Operator
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
                .toggleOnTrue(Commands.runOnce(() -> m_intakeSubsystem.setPivotPosition(0), m_intakeSubsystem))
                .toggleOnFalse(Commands.runOnce(() -> m_intakeSubsystem.stopPivot(), m_intakeSubsystem));

        m_operatorController.circle()
                .toggleOnTrue(Commands.runOnce(() -> m_intakeSubsystem.setPivotPosition(19), m_intakeSubsystem))
                .toggleOnFalse(Commands.runOnce(() -> m_intakeSubsystem.stopPivot(), m_intakeSubsystem));

        m_operatorController.R1().whileTrue(shootToggleCommand);
    }

    /**
     * Blue Left 2-Cycle
     * BlueLeft_Collect → BlueLeft_Return → BlueLeft_Collect → BlueLeft_Return
     * The Return trajectory ends exactly where Collect starts, so they chain cleanly.
     */
    private AutoRoutine blueLeft2Cycle() {
        AutoRoutine routine = m_autoFactory.newRoutine("blueLeft2Cycle");

        AutoTrajectory collect1 = routine.trajectory("BlueLeft_Collect");
        AutoTrajectory return1  = routine.trajectory("BlueLeft_Return");
        AutoTrajectory collect2 = routine.trajectory("BlueLeft_Collect");
        AutoTrajectory return2  = routine.trajectory("BlueLeft_Return");

        // When the routine becomes active, reset odometry then drive the first segment.
        routine.active().onTrue(Commands.sequence(
                collect1.resetOdometry(),
                collect1.cmd()
        ));

        collect1.done().onTrue(return1.cmd());
        return1.done().onTrue(collect2.cmd());
        collect2.done().onTrue(return2.cmd());

        return routine;
    }

    /**
     * Blue Right 2-Cycle
     * BlueRight_Collect → BlueRight_Return → BlueRight_Collect → BlueRight_Return
     */
    private AutoRoutine blueRight2Cycle() {
        AutoRoutine routine = m_autoFactory.newRoutine("blueRight2Cycle");

        AutoTrajectory collect1 = routine.trajectory("BlueRight_Collect");
        AutoTrajectory return1  = routine.trajectory("BlueRight_Return");
        AutoTrajectory collect2 = routine.trajectory("BlueRight_Collect");
        AutoTrajectory return2  = routine.trajectory("BlueRight_Return");

        routine.active().onTrue(Commands.sequence(
                collect1.resetOdometry(),
                collect1.cmd()
        ));

        collect1.done().onTrue(return1.cmd());
        return1.done().onTrue(collect2.cmd());
        collect2.done().onTrue(return2.cmd());

        return routine;
    }

    /**
     * Blue Middle → Left Cycle
     * BlueMiddle_Go_L → BlueLeft_Return → BlueLeft_Collect → BlueLeft_Return
     */
    private AutoRoutine blueMiddleToLeftCycle() {
        AutoRoutine routine = m_autoFactory.newRoutine("blueMiddleToLeftCycle");

        AutoTrajectory goLeft   = routine.trajectory("BlueMiddle_Go_L");
        AutoTrajectory return1  = routine.trajectory("BlueLeft_Return");
        AutoTrajectory collect  = routine.trajectory("BlueLeft_Collect");
        AutoTrajectory return2  = routine.trajectory("BlueLeft_Return");

        routine.active().onTrue(Commands.sequence(
                goLeft.resetOdometry(),
                goLeft.cmd()
        ));

        goLeft.done().onTrue(return1.cmd());
        return1.done().onTrue(collect.cmd());
        collect.done().onTrue(return2.cmd());

        return routine;
    }

    /**
     * Blue Middle → Right Cycle
     * BlueMiddle_Go_R → BlueRight_Return → BlueRight_Collect → BlueRight_Return
     */
    private AutoRoutine blueMiddleToRightCycle() {
        AutoRoutine routine = m_autoFactory.newRoutine("blueMiddleToRightCycle");

        AutoTrajectory goRight  = routine.trajectory("BlueMiddle_Go_R");
        AutoTrajectory return1  = routine.trajectory("BlueRight_Return");
        AutoTrajectory collect  = routine.trajectory("BlueRight_Collect");
        AutoTrajectory return2  = routine.trajectory("BlueRight_Return");

        routine.active().onTrue(Commands.sequence(
                goRight.resetOdometry(),
                goRight.cmd()
        ));

        goRight.done().onTrue(return1.cmd());
        return1.done().onTrue(collect.cmd());
        collect.done().onTrue(return2.cmd());

        return routine;
    }

    //Helpers
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
        return Commands.none();
    }

    public void onAutonomousInit() {
        m_swerveSubsystem.resetMode();
        m_swerveSubsystem.setScaleInput(false);
    }

    public void onTeleopInit() {
        m_swerveSubsystem.resetMode();
        m_swerveSubsystem.setScaleInput(false);
    }

    public Runnable dashboardLoop() {
        return () -> {};
    }
}