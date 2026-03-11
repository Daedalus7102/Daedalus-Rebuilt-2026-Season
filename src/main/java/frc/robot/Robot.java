// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in the TimedRobot documentation.
 *
 * NOTE: With ChoreoLib's AutoChooser, you no longer need to call
 * getAutonomousCommand() here.  AutoChooser binds itself to
 * RobotModeTriggers.autonomous() inside RobotContainer, so the selected
 * routine starts / stops automatically.
 */
public class Robot extends TimedRobot {

    private final RobotContainer m_robotContainer;
    private double m_lastLoopTimestampSeconds = 0.0;
    private double m_lastLoopDiagPublishSeconds = 0.0;
    private int    m_loopOverrunCount = 0;

    private static final double kLoopOverrunThresholdSeconds = 0.022;
    private static final double kLoopDiagPublishPeriodSeconds = 0.5;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    /**
     * robotPeriodic runs every 20 ms in every mode.
     * CommandScheduler.run() must be called here for the entire
     * command-based framework (including ChoreoLib triggers) to work.
     */
    @Override
    public void robotPeriodic() {
        double nowSeconds    = Timer.getFPGATimestamp();
        double loopDtSeconds = nowSeconds - m_lastLoopTimestampSeconds;

        if (m_lastLoopTimestampSeconds > 0.0 && loopDtSeconds > kLoopOverrunThresholdSeconds) {
            m_loopOverrunCount++;
        }
        m_lastLoopTimestampSeconds = nowSeconds;

        // This runs all scheduled commands, polls buttons, and calls subsystem
        // periodic() methods.  ChoreoLib's AutoChooser relies on this too.
        CommandScheduler.getInstance().run();

        if (nowSeconds - m_lastLoopDiagPublishSeconds >= kLoopDiagPublishPeriodSeconds) {
            SmartDashboard.putNumber("RobotLoopDtMs",    loopDtSeconds * 1000.0);
            SmartDashboard.putNumber("RobotLoopOverruns", m_loopOverrunCount);
            m_lastLoopDiagPublishSeconds = nowSeconds;
        }
    }

    @Override public void disabledInit()    {}
    @Override public void disabledPeriodic(){}

    @Override
    public void autonomousInit() {
        // AutoChooser already handles scheduling via RobotModeTriggers.autonomous().
        // We only need the subsystem-level init (reset drive mode, etc.).
        m_robotContainer.onAutonomousInit();
    }

    @Override public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        m_robotContainer.onTeleopInit();
    }

    @Override public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override public void testPeriodic()       {}
    @Override public void simulationInit()     {}
    @Override public void simulationPeriodic() {}
}