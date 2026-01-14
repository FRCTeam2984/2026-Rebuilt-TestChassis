// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Driver_Controller;
import frc.robot.Vision;

public class Robot extends TimedRobot {
  public static int scoringPos = 0;
  private Command m_autonomousCommand;
  private Vision vision;
  public final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }
  @Override
  public void robotInit() {
    //vision = new Vision(RobotContainer.drivetrain::addVisionMeasurement);
  }
  
  @Override
  public void robotPeriodic() {
    Driver_Controller.SwerveInputPeriodic();
    CommandScheduler.getInstance().run(); 
    //vision.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    Driver_Controller.define_Controller();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    AutoDrive.driveSpline(2.0);
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    Driver_Controller.define_Controller();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.rotaryCalc(true);
    Driver_Controller.SwerveControlSet(false);
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    


  }

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
