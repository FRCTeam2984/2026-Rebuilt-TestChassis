// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {
  public static Boolean isTestChassis = false;
  public static char alliance = 'B';
  public static int scoringPos = 0;
  public static boolean autoLastPressed = false;
  private Command m_autonomousCommand;
  private Vision vision, vision2;
  public final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }
  @Override
  public void robotInit() {
    LED.assignPort();
    LED.giveRange('d');
    Turret.readFiles();
    Transport.initMotors();
    Turret.initMotors();
    Constants.Vision.readLayout();
    vision = new Vision(RobotContainer.drivetrain::addVisionMeasurement, new PhotonCamera(Constants.Vision.kCameraName), new PhotonPoseEstimator(Constants.Vision.kTagLayout, Constants.Vision.kRobotToCam));
    vision2 = new Vision(RobotContainer.drivetrain::addVisionMeasurement, new PhotonCamera(Constants.Vision.kCamera2Name), new PhotonPoseEstimator(Constants.Vision.kTagLayout, Constants.Vision.kRobotToCam2));
  }
  
  @Override
  public void robotPeriodic() {
    alliance = DriverStation.getAlliance().toString().charAt(9);
    Driver_Controller.SwerveInputPeriodic();
    CommandScheduler.getInstance().run(); 
    vision.periodic();
    vision2.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if ((vision.camera.isConnected() && vision2.camera.isConnected()) == false){
      LED.setPattern('e');
    }else if (vision.seenTags || vision2.seenTags){
      LED.setPattern("rainbow");
    }else{
      LED.setPattern("warn");
    }
  }

  @Override
  public void disabledExit() {
    LED.setPattern("rainbow");
  }

  public static Double[] destPoints = {0.0, 0.0, 0.0}; // x1, x2, y
  public static void determinePoints(){
    Double odoy = RobotContainer.drivetrain.getState().Pose.getY();
        Double odox = RobotContainer.drivetrain.getState().Pose.getX();
    if (odoy > 4.034663){
      destPoints[2] = +7.4247756+0.1;
    }else{
      destPoints[2] = (4.034663*2-7.4247756)-0.1;
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
  String auto = "stay, shoot";
  
  @Override
  public void autonomousInit() {
    RobotContainer.rotaryCalc(true);
    Autonomous.reset();
    AutoDrive.alliance = DriverStation.getAlliance().toString().charAt(9);
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    Driver_Controller.define_Controller();
    if (m_autonomousCommand != null) {
      //m_autonomousCommand.schedule();
    }
    Turret.curPower[0] = 0.0; Turret.curPower[1] = 0.0;
    auto = RobotContainer.autoChooser.getSelected();
    Autonomous.idleShooter = RobotContainer.idleShooterSelector.getSelected();
    Autonomous.shootFirst = RobotContainer.shootFirstSelector.getSelected();
    Autonomous.reset();
  }

  @Override
  public void autonomousPeriodic() {
    switch(auto){
      case "shuttle":
        Autonomous.shuttleAuto();
        break;
      case "outpost":
        Autonomous.outpostAuto();
        break;
      case "half-intake":
        Autonomous.halfIntakeAuto();
        break;
      case "half+return":
        Autonomous.halfIntakeReturnAuto();
        break;
      case "intake, shoot":
        Autonomous.intakeAuto();
        break;
      case "depot":
        Autonomous.depotAuto();
        break;
      case "stay, shoot":
        Driver_Controller.SwerveCommandEncoderValue = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();// pls work
        Driver_Controller.SwerveCommandXValue = 0.0;
        Driver_Controller.SwerveCommandYValue = 0.0;
        Driver_Controller.SwerveControlSet(true);
      default:
        Autonomous.shootAuto();
        break;
    }
  }

  @Override
  public void autonomousExit() {}

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

  public static BufferedWriter fileWriter;
  @Override
  public void teleopInit() {
    try {fileWriter = new BufferedWriter(new FileWriter("/home/admin/errorData.txt"));}catch (Exception e) {e.printStackTrace();}
    cnt = 0;
    AutoDrive.alliance = DriverStation.getAlliance().toString().charAt(9);
    System.out.println(DriverStation.getAlliance().toString());
    Driver_Controller.define_Controller();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.rotaryCalc(true);
    Driver_Controller.SwerveControlSet(false);
    Turret.curPower[0] = 0.0; Turret.curPower[1] = 0.0;
  }

  public static int cnt = 0;
  public static int autoState;
  Double angle = 45.0;

  static Double[] curCowlSlider = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  static Double[] curSpeedSlider = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  static Double[] curOffsetSlider = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  public static int curIndexCowl = 0, curIndexSpeed = 0, curOffsetIndex = 0;

  static Boolean saveLastPressed = false, recentClose = false, recentPressed = false, startedOutRange = false;
  public static Double prevDesired = 0.0;

  @Override
  public void teleopPeriodic() {
    SignalLogger.stop();
    if (System.nanoTime()%100*1000*1000 > 70*1000*1000)Driver_Controller.kitchenStove();
    Double transportPower, intakePower, spinPower, spindexPow;
    Turret.calcDist();

    // find the average of sliders over past 0.2 sec
    curCowlSlider[curIndexCowl] = Driver_Controller.cowlSlider();
    curSpeedSlider[curIndexSpeed] = Driver_Controller.shooterSpeedSlider();
    curOffsetSlider[curOffsetIndex] = Driver_Controller.offsetSlider();
    Double averageSpeed = 0.0, averageCowl = 0.0, averageOffset = 0.0;
    for (int i = 0; i < 10; ++i){
      averageSpeed += curSpeedSlider[i]/10;
      averageCowl += curCowlSlider[i]/10;
      averageOffset += curOffsetSlider[i]/10;
    }
    curIndexSpeed = (curIndexSpeed+1)%10;
    curIndexCowl = (curIndexCowl+1)%10;
    curOffsetIndex = (curOffsetIndex+1)%10;

    // reset the turret/get power
    if (Driver_Controller.buttonResetTurret()){
      Turret.encoderStatus = 's';
    }
    spinPower = Turret.resetEncoder();
    if (Math.abs(spinPower) < 0.0001){
      Turret.encoderStatus = 'b';
      spinPower = Turret.spinTurret();
    }
    if (Driver_Controller.pauseTurret()) spinPower = 0.0;

    // calc power for the transport and stuff
    intakePower = Transport.spinIntake();
    spindexPow = Transport.spindexerPower();
    transportPower = Transport.spinTransport();

    // if (Turret.inAlliance)
    // Turret.desiredSpeed = Math.min(Math.max(10, 60-20*(Turret.distance-2.5)), Turret.desiredSpeed);
    // get shooter power and set LEDs
    if (Driver_Controller.manualSwitch())Turret.desiredSpeed = averageSpeed;
    if (Driver_Controller.manualSwitch())Turret.cowlAngle = averageCowl;
    if (Driver_Controller.manualSwitch())Turret.interpolatedTurretOffset = averageOffset;
    Double[] power = {0.0, 0.0};
    recentClose |= Turret.close;
    if (Driver_Controller.runShooterSwitch()){
      if (Turret.distance > 2.5){
        if (!recentPressed) startedOutRange = true;
        LED.giveRange("far");
        if (!startedOutRange){
          recentClose = false;
          transportPower = 0.0;
          spindexPow  = 0.0;
        }
      }else{
        LED.giveRange("close");
      }
      power = Turret.speedController();
    }else{
      if (!recentPressed) startedOutRange = false;
      recentClose = false;
      LED.giveRange("hide");
      Turret.avgSpeed = 0.0;
      if (Driver_Controller.buttonShooterReverse()){
        power[0] = -0.15;
        power[1] = -0.15;
      }
      Turret.curPower[0] = 0.0;
      Turret.curPower[1] = 0.0;
      if (vision.seenTags || vision2.seenTags){
        LED.setPattern("rainbow");
      }else{
        LED.setPattern("warn");
      }
    }
    recentPressed = Driver_Controller.runShooterSwitch();

    // restrict transport if in alliance zone, a toggle, and fast9
    Double robotVeloX = RobotContainer.drivetrain.getState().Speeds.vxMetersPerSecond;
    Double robotVeloY = RobotContainer.drivetrain.getState().Speeds.vyMetersPerSecond;
    Double fieldDriveVeloAngle = Math.toDegrees(Math.atan2(robotVeloY, robotVeloX));
    if (Driver_Controller.buttonRestrictTransport() && Turret.inAlliance && ((((Math.sin(Math.toRadians(Turret.targetAngleDeg-fieldDriveVeloAngle))
      *Math.sqrt(Math.pow(robotVeloX , 2) + Math.pow(robotVeloY, 2))) > 0.5)  // driving away from hub
      || (Math.sqrt(Math.pow(robotVeloX , 2) + Math.pow(robotVeloY, 2)) > 1.0)
      || Math.abs(RobotContainer.drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 90)
      || recentClose == false)){  // driving any direction
        transportPower = 0.0;
        spindexPow  = 0.0;
    }

    // periodically (every 10 cycles) print out current data
    if (curIndexCowl == 0){
      System.out.printf("dist=%.3f, cowl=%.3f, shooter=%.3f, offset=%.3f, angle=%.3f, speed=%.3f\n", Turret.distance, Turret.cowlAngle, Turret.desiredSpeed, Turret.interpolatedTurretOffset, Turret.targetAngleDeg, Turret.avgSpeed);
    }

    // ++cnt;
    // Double rate = prevDesired-Turret.targetAngleDeg, error = Turret.modTurretOff;
    // try{fileWriter.write(cnt+", "+rate+", "+error);fileWriter.newLine();}catch(Exception e){System.out.println("fail");e.printStackTrace();};
    prevDesired = Turret.targetAngleDeg;
    // Turret.servo1.set((1.0+Driver_Controller.m_Controller3.getRawAxis(4))/2.0);
    // Turret.servo2.set((1.0+Driver_Controller.m_Controller3.getRawAxis(3))/2.0);
    // Turret.servoInverted.set((1.0+Driver_Controller.m_Controller3.getRawAxis(1))/2.0);

    // set power to motors if not EBrake
    if (Driver_Controller.buttonEBrake()){
      Transport.setIntake(0.0);
      Transport.setTransport(0.0);
      Transport.setSpindexer(0.0);
      Turret.shooter1.set(0.0);
      Turret.shooter2.set(0.0);
      Turret.turretSpin.set(0.0);
    }else{
      Transport.setIntake(intakePower);
      Transport.setTransport(-transportPower);
      Transport.setSpindexer(spindexPow);
      Turret.shooter1.set(power[0]);
      Turret.shooter2.set(-power[1]);
      Turret.turretSpin.set(spinPower);
      Turret.servo1.set(Turret.cowlAngle); // axis 4 at +0.29 -> .645 
      Turret.servo2.set(Turret.cowlAngle-0.24); // axis 3 at -0.19 -> .405 Servoangle-.24
      Turret.servoInverted.set(1.275-Turret.cowlAngle); // axis 1 at +0.26 -> .63, 1.275-Servoangle
    }
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