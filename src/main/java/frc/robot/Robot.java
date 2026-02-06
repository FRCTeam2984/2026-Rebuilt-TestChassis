// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.MatchType;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.*;
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
    LED.setPattern("rainbow");
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
    Turret.curPower[0] = 0.0; Turret.curPower[1] = 0.0;
  }

  @Override
  public void autonomousPeriodic() {
    AutoDrive.driveSpline();
  }

  @Override
  public void autonomousExit() {
    LED.setPattern("rainbow");
  }

  public static Integer activeTime(){
    Double time = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage(); // alliance with first inactive shift
    // switch(gameData){
    //   case "B":
    //     if((time/30)%2==0){

    //     }
    //     break;
    //   case "R":
    //     break;
    //   default:
        if(alliance==gameData.charAt(0)){//if our alliance is the same with first inactive shift
          if((time/30)%2==0){
            return(time%30);
          }else{
            return(-time%30);
          }
     
        }else{
          if((time/30)%2==1){
            return(time%30);
          }else{
            return(-time%30);
          }
        }
        return 0;
    }


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
    Turret.curPower[0] = 0.0; Turret.curPower[1] = 0.0;
  }

  static int cnt = 0;

  @Override
  public void teleopPeriodic() {
    //Turret.calcDist();
    /*if (Driver_Controller.buttonReefAlign() && false){
      if (autoLastPressed){
        Double odoy = RobotContainer.drivetrain.getState().Pose.getY();
        Double odox = RobotContainer.drivetrain.getState().Pose.getX();
        AutoDrive.setSpline(odox, odoy, 8.8, 4.0, 0.0, 0.0, 3.0, 50);
    
      }
      AutoDrive.driveSpline();
    }else Driver_Controller.SwerveControlSet(false);
    autoLastPressed = Driver_Controller.buttonReefAlign();
    Transport.moveMotor();*/
    if (true){ // change to true to become shooter code
      Double[] power = Turret.speedController();
    Turret.shooter1.set(1+0*power[0]);
    Turret.shooter2.set(-power[1]*0+1);
    Turret.transportMotor.set(TalonSRXControlMode.PercentOutput, -0.2);
    }else{
      if (Driver_Controller.buttonL2())Turret.servo1.setAngle(0);
      else Turret.servo1.setAngle(180);
      if (Driver_Controller.buttonL2())Turret.servo2.setAngle(0);
      else Turret.servo2.setAngle(180);
      if (Driver_Controller.buttonL2())Turret.servoInverted.setAngle(180);
      else Turret.servoInverted.setAngle(0);
    }
  }

  @Override
  public void teleopExit() {
    LED.setPattern("rainbow");
  }

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
