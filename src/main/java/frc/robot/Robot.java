// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Driver_Controller;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Transport;
import frc.robot.subsystems.Turret;
import frc.robot.Vision;

public class Robot extends TimedRobot {
  public static char alliance = 'B';
  public static int scoringPos = 0;
  public static boolean autoLastPressed = false;
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
    alliance = DriverStation.getAlliance().toString().charAt(9);
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
    AutoDrive.alliance = DriverStation.getAlliance().toString().charAt(9);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    Driver_Controller.define_Controller();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    AutoDrive.driveSpline();
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    AutoDrive.alliance = DriverStation.getAlliance().toString().charAt(9);
    System.out.println(DriverStation.getAlliance().toString());
    Driver_Controller.define_Controller();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.rotaryCalc(true);
    Driver_Controller.SwerveControlSet(false);
  }

  static int cnt = 0;

  @Override
  public void teleopPeriodic() {
    /*if (Driver_Controller.buttonReefAlign() && false){
      if (autoLastPressed){
        Double odoy = RobotContainer.drivetrain.getState().Pose.getY();
        Double odox = RobotContainer.drivetrain.getState().Pose.getX();
        AutoDrive.setSpline(odox, odoy, 8.8, 4.0, 0.0, 0.0, 3.0, 50);
    
      }
      AutoDrive.driveSpline();
    }else Driver_Controller.SwerveControlSet(false);
    autoLastPressed = Driver_Controller.buttonReefAlign();
    Transport.moveMotor();
    if(Driver_Controller.buttonL2()){
      //Turret.turretSpin.set(0.1);
    } else{
      //Turret.turretSpin.set(0.0);
    }*/
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
