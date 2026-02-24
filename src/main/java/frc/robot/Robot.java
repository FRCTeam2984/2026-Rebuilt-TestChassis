// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Servo;

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
    Transport.initMotors();
    Turret.initMotors();
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
  public static int autoState;
  Double angle = 45.0;
  public static Double[] destPoints = {0.0, 0.0, 0.0}; // x1, x2, y

  public static void determinePoints(){
    Double odoy = RobotContainer.drivetrain.getState().Pose.getY();
        Double odox = RobotContainer.drivetrain.getState().Pose.getX();
    if (odoy > 4.034663){
      destPoints[2] = (4.034663+7.4247756)/2;
    }else{
      destPoints[2] = ((4.034663*3)-7.4247756)/2;
    }
    if (odox > (11.915394+4.62534)/2){
      destPoints[0] = 11.915394;
      if (odox > 11.915394){
        destPoints[1] = 11.915394-2;
      }else{
        destPoints[1] = 11.915394+2;
      }
    }else{
      destPoints[0] = 4.62534;
      if (odox > 4.62534){
        destPoints[1] = 4.62534-2;
      }else{
        destPoints[1] = 4.62534+2;
      }
    }
  }

  public static Double desiredSpeed = 50.0, cowlAngle = 0.5;
  public static Servo servo1 = new Servo(2);

  static Double[] curUpperSlider = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  static Double[] curLowerSlider = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  static int curIndexUpper = 0, curIndexLower = 0;

  @Override
  public void teleopPeriodic() {
    Double transportPower, intakePower;
    // if (Driver_Controller.buttonReefAlign()){
    //   if (!autoLastPressed){
    //     determinePoints();
    //     angle = 1.0*Math.round(RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees()/90)*90+55;
    //     autoState = 0;
    //     Double odoy = RobotContainer.drivetrain.getState().Pose.getY();
    //     Double odox = RobotContainer.drivetrain.getState().Pose.getX();
    //     AutoDrive.setSpline(odox, odoy, destPoints[0], destPoints[2], destPoints[0]-destPoints[1], 0.0, 2.5, 50);
    //     System.out.println(destPoints[0]+"  "+destPoints[1]+"  "+destPoints[2]);
    //   }

    //   if (AutoDrive.driveSpline(angle)){

    //     if (autoState == 0){
    //       Double odoy = RobotContainer.drivetrain.getState().Pose.getY();
    //       Double odox = RobotContainer.drivetrain.getState().Pose.getX();
    //       Driver_Controller.SwerveControlSet(true);
    //       AutoDrive.setSpline(odox, odoy, destPoints[1], destPoints[2], 0.0, 0.0, 2.5, 50);
    //       ++autoState;
    //     }else if (autoState == 1){
    //       ++autoState;
    //       Driver_Controller.SwerveControlSet(false);
    //     }else{
    //       Driver_Controller.SwerveControlSet(false);
    //     }
    //   }
    // }else Driver_Controller.SwerveControlSet(false);

    curUpperSlider[curIndexUpper] = Driver_Controller.upperDriverSlider();
    curLowerSlider[curIndexLower] = Driver_Controller.lowerDriverSlider();
    Double lowerDriverSlider = 0.0, upperDriverSlider = 0.0;
    for (int i = 0; i < 10; ++i){
      lowerDriverSlider += curLowerSlider[i]/10;
      upperDriverSlider += curUpperSlider[i]/10;
    }
    curIndexLower = (curIndexLower+1)%10;
    curIndexUpper = (curIndexUpper+1)%10;

    desiredSpeed = 45+lowerDriverSlider*35;
    cowlAngle = 0.5+upperDriverSlider*0.5;
    //System.out.println(desiredSpeed + ",  " + cowlAngle);
    Turret.calcDist();
    //autoLastPressed = Driver_Controller.buttonReefAlign();
    if (Driver_Controller.buttonL1()){
      Turret.turretSpin.set(Turret.resetEncoder());
    }else if (Driver_Controller.buttonScoreAlgae()){
      Turret.turretSpin.set(Turret.spinTurret()/2);
      Turret.encoderStatus = 's';
    }else{
      Turret.turretSpin.set(0.0);
    }
    Double[] power = {0.0, 0.0};
    if (Driver_Controller.buttonL2()){
      power = Turret.speedController();
    }else{
      Turret.curPower[0] = 0.0;
      Turret.curPower[1] = 0.0;
    }
    Turret.servo1.set(cowlAngle);
    Turret.servo2.set(cowlAngle);
    Turret.servoInverted.set(1-cowlAngle);

    Turret.shooter1.set(power[0]);
    Turret.shooter2.set(-power[1]);

    intakePower = Transport.spinIntake();
    transportPower = Transport.spinTransport();

    Transport.setIntake(intakePower);
    Transport.setTransport(transportPower);
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
