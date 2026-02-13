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
  public static Boolean isTestChassis = false;
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
    LED.assignPort();
    LED.setPattern("rainbow");
    Turret.readFiles();
    Constants.Vision.readLayout();
    vision = new Vision(RobotContainer.drivetrain::addVisionMeasurement);
  }
  
  @Override
  public void robotPeriodic() {
    alliance = DriverStation.getAlliance().toString().charAt(9);
    Driver_Controller.SwerveInputPeriodic();
    CommandScheduler.getInstance().run(); 
    vision.periodic();
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
    Integer time = Math.toIntExact(Math.round(DriverStation.getMatchTime()));
    String gameData = DriverStation.getGameSpecificMessage(); // alliance with first inactive shift
    if(time < 30){ // endgame
      if (gameData == ""){
        return time+10; // time until transition period ends
      }
      return time; // time until match ends
    }
    if(time < 55){ // last alliance shift
      if(gameData == ""){
        return 30-time; // assume it is inactive, time (negative/inactive) until endgame
      }else if (gameData.charAt(0) == alliance){
        return time; // time until match ends, because after current alliance shift, it is endgame
      }else{
        return 30-time; // time until endgame, is negative/inactive
      }
    }
    if(time < 80){ // third alliance shift
      if(gameData == ""){
        return 30-time; // assume it is inactive, time (negative) until endgame
      }else if (gameData.charAt(0) == alliance){
        return 55-time; // time until last alliance shift, is negative because is inactive
      }else{
        return time-55; // time until last alliance shift because it is active now
      }
    }
    if(time < 105){ // second alliance shift
      if(gameData == ""){
        return 30-time;
      }else if (gameData.charAt(0) == alliance){
        return time-80;
      }else{
        return 80-time;
      }
    }
    if(time < 130){ // first alliance shift
      if(gameData == ""){
        return 30-time;
      }else if (gameData.charAt(0) == alliance){
        return 105-time;
      }else{
        return time-105;
      }
    }
    // transition shift
    if(gameData == ""){
      return time-130;
    }else if (gameData.charAt(0) == alliance){
      return time-130;
    }else{
      return time-105;
    }
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
    /*if (Driver_Controller.buttonReefAlign()){
      if (!autoLastPressed){
        Double odoy = RobotContainer.drivetrain.getState().Pose.getY();
        Double odox = RobotContainer.drivetrain.getState().Pose.getX();
        AutoDrive.setSpline(odox, odoy, odox+1, odoy, 0.0, 0.0, 1.0, 50);
    
      }
      AutoDrive.driveSpline();
      //System.out.println(Driver_Controller.SwerveXPassthrough);
      //System.out.println(Driver_Controller.SwerveYPassthrough);
    }else Driver_Controller.SwerveControlSet(false);
    autoLastPressed = Driver_Controller.buttonReefAlign();*/
    // if (Driver_Controller.buttonL1()){
    //   Turret.resetEncoder();
    //   System.out.println(Turret.resettingSensor.get());
    // }else{
      Turret.turretSpin.set(0.0);
    // }
    Double[] power = {0.0, 0.0};
    if (Driver_Controller.buttonL2()){
      power = Turret.speedController();
    }else{
      Turret.curPower[0] = 0.0;
      Turret.curPower[1] = 0.0;
    }
    Turret.shooter1.set(power[0]);
    Turret.shooter2.set(-power[1]);
    Transport.spinIntake();
    Transport.spinTransport();
    //if(Driver_Controller.buttonL2())Turret.resetEncoder();
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
