// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@Logged(strategy = Strategy.OPT_IN)
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private boolean hasPopulatedTestModeDashboard = false;

  @Logged
  private final RobotContainer m_robotContainer;
  private final Timer logTimer = new Timer();

  /* log and replay timestamp and joystick data */
  private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay().withTimestampReplay()
      .withJoystickReplay();

  public Robot() {
    m_robotContainer = new RobotContainer();
    logTimer.start();

    SignalLogger.start(); // CTRE logger
    DataLogManager.start(); // WPILib logger
    DriverStation.startDataLog(DataLogManager.getLog()); // Record both DS control and joystick data
    Epilogue.bind(this);

  }

  @Override
  public void robotInit() {
    // Webserver for Elastic layout
    // See https://frc-elastic.gitbook.io/docs/additional-features-and-references/remote-layout-downloading
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
  }

  @Override
  public void robotPeriodic() {
    m_timeAndJoystickReplay.update();
    CommandScheduler.getInstance().run();

    if (logTimer.advanceIfElapsed(1)) {
      DataLogManager.getLog().resume();
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    if (!hasPopulatedTestModeDashboard) {
      m_robotContainer.populateTestModeDashboard();
      hasPopulatedTestModeDashboard = true;
    }
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
