// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final Runnable m_dashboardLoop;
  private double m_lastLoopTimestampSeconds = 0.0;
  private double m_lastLoopDiagPublishSeconds = 0.0;
  private int m_loopOverrunCount = 0;

  private static final double kLoopOverrunThresholdSeconds = 0.022;
  private static final double kLoopDiagPublishPeriodSeconds = 0.5;

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_dashboardLoop = m_robotContainer.dashboardLoop();
  }

  @Override
  public void robotPeriodic() {
    double nowSeconds = Timer.getFPGATimestamp();
    double loopDtSeconds = nowSeconds - m_lastLoopTimestampSeconds;
    if (m_lastLoopTimestampSeconds > 0.0 && loopDtSeconds > kLoopOverrunThresholdSeconds) {
      m_loopOverrunCount++;
    }
    m_lastLoopTimestampSeconds = nowSeconds;

    CommandScheduler.getInstance().run();
    m_dashboardLoop.run();

    // ── LED UPDATE ──
    boolean isAuto = DriverStation.isAutonomous() && DriverStation.isEnabled();
    boolean isEndgame = DriverStation.isTeleop() && DriverStation.isEnabled()
        && DriverStation.getMatchTime() >= 0
        && DriverStation.getMatchTime() <= 30;
    m_robotContainer.updateLEDs(isAuto, isEndgame);
    // ────────────────

    if (nowSeconds - m_lastLoopDiagPublishSeconds >= kLoopDiagPublishPeriodSeconds) {
      SmartDashboard.putNumber("RobotLoopDtMs", loopDtSeconds * 1000.0);
      SmartDashboard.putNumber("RobotLoopOverruns", m_loopOverrunCount);
      m_lastLoopDiagPublishSeconds = nowSeconds;
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.onAutonomousInit();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_robotContainer.onTeleopInit();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}