package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.RobotContainer;

// this is mostly organization for our different autos
public class Autonomous {
    public static char alliance;
    public static Boolean ready, idleShooter = true;
    public static Integer revdIntakeCnt = 0;
    public static Double bigSpeed = 0.0, prevSpeed = 0.0;
    public static Integer prev_autostate = -1;
    public static void shootAuto(Boolean transport){ // this function automatically shoots
        if (revdIntakeCnt <= -1){
            if (revdIntakeCnt == 0) prevSpeed = Transport.prevIntakePower;
            Transport.setIntake(-0.4);
            if (revdIntakeCnt == -1) Transport.setIntake(prevSpeed);
        }
        ++revdIntakeCnt;
        Turret.calcDist();
        Double shooterPower = Turret.resetEncoder();
        if (Math.abs(shooterPower) < 0.0001){
            shooterPower = Turret.spinTurret();
        }
        if (Driver_Controller.pauseTurret()) shooterPower = 0.0;
        Turret.turretSpin.set(shooterPower);
        if ((idleShooter == false) && (transport == false)){
            ready = false;
            Turret.shooter1.set(0.0);
            Turret.shooter2.set(0.0);
            Transport.setTransport(0.0);
            Transport.setSpindexer(0.0);
            Transport.agitate(false);
            return;
        }
        // spin up shooter and reset/aim turret and stuff
        
        if (transport == false) Turret.desiredSpeed = 20.0;
        Double[] power = Turret.speedController();
        Turret.shooter1.set(power[0]);
        Turret.shooter2.set(-power[1]);
        bigSpeed = Math.max(bigSpeed, Turret.avgSpeed);
        System.out.printf("desired=%.3f, cur=%.3f, max=%.3f\n", Turret.desiredSpeed, Turret.avgSpeed, bigSpeed);
        Turret.servo1.set(Turret.cowlAngle);
        Turret.servo2.set(Turret.cowlAngle-0.129);
        Turret.servoInverted.set(1-Turret.cowlAngle+0.162);
        // checks everything if ready to shoot
        ready |= (Turret.close
            && (Driver_Controller.pauseTurret() || (Math.abs(Turret.modTurretOff) < 5)));
        if (ready && transport){
            Transport.setSpindexer(Transport.powerArray[Driver_Controller.kitchenStove()]);
            Transport.setTransport(0.85);
            Transport.agitatorMotor.set(TalonSRXControlMode.PercentOutput, 0.8);
        }else{
            Transport.setTransport(0.0);
            Transport.setSpindexer(0.0);
            Transport.agitate(false);
        }
    }
    public static void shootAuto(){
        shootAuto(true);
    }

    public static Boolean shouldSkipMove = true;
    public static Double[] destPoints = {0.0, 0.0, 0.0}; // x1, x2, y
    public static void enterNeutralPoints(){
        shouldSkipMove = false;
        Double odoy = RobotContainer.drivetrain.getState().Pose.getY();
        Double odox = RobotContainer.drivetrain.getState().Pose.getX();
        if (odoy > 4.034663){
            destPoints[2] = 7.4247756;//-0.15;
        }else{
            destPoints[2] = (4.034663*2-7.4247756);//+0.15;
        }
        if (odox > (11.915394+4.62534)/2){
            destPoints[0] = 11.915394+1.0;
            if (odox < (11.915394+0.8)){
                shouldSkipMove = true;
            }
            destPoints[1] = 11.915394-2;
        }else{
            destPoints[0] = 4.62534-1.0;
            if (odox > (4.62534-0.8)){
                shouldSkipMove = true;
            }
            destPoints[1] = 4.62534+2;
        }
    }

    public static void enterAlliancePoints(){
        Double odoy = RobotContainer.drivetrain.getState().Pose.getY();
        if (odoy > 4.034663){
            destPoints[2] = 7.4247756;//-0.075;
        }else{
            destPoints[2] = (4.034663*2-7.4247756);//+0.075;
        }
        
        if (alliance == 'R'){
            destPoints[0] = 11.915394;
            destPoints[1] = 11.915394+1.2;
        }else{
            destPoints[0] = 4.62534;
            destPoints[1] = 4.62534-1.2;
        }
    }

    public static Double speed = 4.0, endVeloMult = 0.3*speed, shootTimeSec = 5.0;
    public static Integer autoState = 0;
    public static void shuttleAuto(){
        ++cnt;
        shootAuto((autoState > 2) || autoState == 0);
        Double driveAngle = ((alliance == 'R')?80.0:100.0)*((destPoints[2] > 4.034663)?1.0:-1.0);
        switch(autoState){
            case 0:
                if (cnt >= 50*shootTimeSec){
                    startIntakeAngle = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    ++autoState;
                }
                break;
            case 1:
                enterNeutralPoints();
                AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                ++autoState;
            case 2:
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    AutoDrive.setSpline(destPoints[1], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                    Transport.setIntake(0.6);
                }
                break;
            case 3:
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    AutoDrive.setSpline((14.552041+1.988947)/2, 4.034663-(destPoints[2]-4.034663)*0.6, 0.0, (destPoints[2]-4.034663)/2*endVeloMult, speed, 50);
                    ++autoState;
                }
                break;
            case 4:
                if (AutoDrive.driveSpline(-driveAngle)){
                    Double xAdd = (alliance == 'R')?1.0:-1.0;
                    AutoDrive.setSpline((14.552041+1.988947)/2+xAdd, 4.034663+(destPoints[2]-4.034663)*0.6, 0.0, -(destPoints[2]-4.034663)/2*endVeloMult, speed, 50);
                    ++autoState;
                }
                break;
            case 5:
                if (AutoDrive.driveSpline(driveAngle)){
                    Double xAdd = (alliance == 'R')?2.0:-2.0;
                    AutoDrive.setSpline((14.552041+1.988947)/2+xAdd, 4.034663-(destPoints[2]-4.034663)*0.6, 0.0, (destPoints[2]-4.034663)/2*endVeloMult, speed, 50);
                    ++autoState;
                }
                break;
            case 6:
                if (AutoDrive.driveSpline(-driveAngle)){
                    ++autoState;
                }
                break;
            case 7:
                Transport.setIntake(0.0);
                Driver_Controller.SwerveCommandEncoderValue = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                Driver_Controller.SwerveCommandXValue = 0.0;
                Driver_Controller.SwerveCommandYValue = 0.0;
                Driver_Controller.SwerveControlSet(true);
                break;
        }
    }

    public static void outpostAuto(){
        Double outpostA = ((alliance == 'R')?0.0:180);
        ++cnt;
        switch(autoState){
            case 0:
                if (cnt >= 50*shootTimeSec){ // 5 seconds
                    Double outpostX = ((alliance == 'R')?11.915394+4.62534-0.5:0.5),
                    outpostY = ((alliance == 'R')?(4.034663*2-0.5):0.5);
                    AutoDrive.setSpline(outpostX, outpostY, 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                shootAuto(true);
                break;
            case 1:
                shootAuto(false);
                Double odoY = RobotContainer.drivetrain.getState().Pose.getY();
                Double odoX = RobotContainer.drivetrain.getState().Pose.getX();
                if (AutoDrive.driveSpline(outpostA) ||
                    ((alliance == 'R')?odoX>(11.915394+4.62534-0.5):odoX<0.5) ||
                    ((alliance == 'R')?odoY>(4.034663*2-0.5):odoY<0.5)){
                        ++autoState;
                        cnt = 0;
                }
                break;
            case 2:
                shootAuto(false);
                Driver_Controller.SwerveControlSet(false);
                if (cnt >= 50*2){ // 2 seconds wait for fuel from outpost
                    Double desiredX = ((alliance == 'R')?
                        Turret.redTargetX[2]+1.2:
                        Turret.blueTargetX[2]-1.2),
                    desiredY = Turret.TargetY[2] +
                        ((alliance == 'R')?1.2:-1.2);
                    AutoDrive.setSpline(desiredX, desiredY, 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 3:
                if (AutoDrive.driveSpline(outpostA)){
                    ++autoState;
                }
                break;
            case 4:
                Driver_Controller.SwerveCommandEncoderValue = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                Driver_Controller.SwerveCommandXValue = 0.0;
                Driver_Controller.SwerveCommandYValue = 0.0;
                Driver_Controller.SwerveControlSet(true);
                shootAuto(true);
                break;
        }
    }

    public static void intakeAuto(){
        ++cnt;
        Double driveAngle = ((destPoints[2] > 4.034663)?-1.0:1.0)*((alliance == 'R')?80.0:100.0);
        switch(autoState){
            case 0:
                if (cnt >= 50*shootTimeSec){
                    startIntakeAngle = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    enterNeutralPoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                shootAuto(true);
                break;
            case 1:
                shootAuto(false);
                if (shouldSkipMove || AutoDrive.driveSpline(startIntakeAngle)){
                    Transport.setIntake(0.6);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 2:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    Double yPosition = 4.034663+((destPoints[2] > 4.034663)?-1.0:1.0);
                    AutoDrive.setSpline((14.552041+1.988947)/2, yPosition, 0.0, (destPoints[2]-4.034663)/2*endVeloMult, speed, 50);
                    ++autoState;
                }
                break;
            case 3:
                shootAuto(false);
                if (AutoDrive.driveSpline(driveAngle)){
                    enterAlliancePoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 4:
                shootAuto(false);
                if (AutoDrive.driveSpline()){
                    Transport.setIntake(0.0);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 5:
                shootAuto(false);
                if (AutoDrive.driveSpline()){
                    Double desiredX = ((destPoints[0] > (14.552041+1.988947)/2)?
                        Turret.redTargetX[2]+1.2:
                        Turret.blueTargetX[2]-1.2),
                    desiredY = Turret.TargetY[2] +
                        ((destPoints[2] > 4.034663)?1.2:-1.2);
                    System.out.println(desiredX);
                    AutoDrive.setSpline(desiredX, desiredY, 0.0, 0.0, speed, 50);
                    Driver_Controller.SwerveCommandEncoderValue = ((destPoints[2] > 4.034663)?-1.0:1.0)*((alliance == 'R')?-50.0:50.0);
                    Driver_Controller.SwerveCommandXValue = 0.0;
                    Driver_Controller.SwerveCommandYValue = 0.0;
                    Driver_Controller.SwerveControlSet(true);
                    ++autoState;
                }
                break;
            case 6:
                shootAuto(true);
                AutoDrive.driveSpline();
                break;
            // case 7:
            //     if (AutoDrive.driveSpline()){
            //         Double desiredX = ((destPoints[0] > (14.552041+1.988947)/2)?
            //             Turret.redTargetX[2]+1.2:
            //             Turret.blueTargetX[2]-1.2),
            //         desiredY = Turret.TargetY[2] +
            //             ((destPoints[2] > 4.034663)?1.2:-1.2);
            //         System.out.println(desiredX);
            //         AutoDrive.setSpline(desiredX, desiredY, 0.0, 0.0, speed, 50);
            //         ++autoState;
            //     }
            //     shootAuto(true);
            //     break;
            // case 8:
            //     if (AutoDrive.driveSpline()){
            //         Double desiredX = ((destPoints[0] > (14.552041+1.988947)/2)?
            //             Turret.redTargetX[2]+1.2:
            //             Turret.blueTargetX[2]-1.2),
            //         desiredY = Turret.TargetY[2] +
            //             ((destPoints[2] > 4.034663)?0.8:-0.8);
            //         System.out.println(desiredX);
            //         AutoDrive.setSpline(desiredX, desiredY, 0.0, 0.0, speed, 50);
            //         --autoState;
            //     }
            //     shootAuto(true);
            //     break;
        }
    }
    
    public static void halfIntakeAuto(){
        ++cnt;
        switch(autoState){
            case 0:
                if (cnt >= 50*shootTimeSec){
                    startIntakeAngle = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    enterNeutralPoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                shootAuto(true);
                break;
            case 1:
                shootAuto(false);
                if (shouldSkipMove || AutoDrive.driveSpline(startIntakeAngle)){
                    Transport.setIntake(0.6);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 2:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    Double yPosition = 4.034663+((destPoints[2] > 4.034663)?0.5:-0.5);
                    AutoDrive.setSpline((14.552041+1.988947)/2+((alliance == 'R')?0.5:-0.5), yPosition, 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 3:
                shootAuto(false);
                if (AutoDrive.driveSpline(Driver_Controller.pigeonOffset+RobotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble())){
                    enterAlliancePoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], 1.5*(destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 4:
                shootAuto(false);
                if (AutoDrive.driveSpline()){
                    Transport.setIntake(0.0);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 5:
                shootAuto(false);
                if (AutoDrive.driveSpline()){
                    Double desiredX = ((destPoints[0] > (14.552041+1.988947)/2)?
                        Turret.redTargetX[2]+1.2:
                        Turret.blueTargetX[2]-1.2),
                    desiredY = Turret.TargetY[2] +
                        ((destPoints[2] > 4.034663)?1.2:-1.2);
                    AutoDrive.setSpline(desiredX, desiredY, 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 6:
                shootAuto(false);
                if (AutoDrive.driveSpline((alliance == 'R')?0.0:180.0)){
                    Driver_Controller.SwerveCommandEncoderValue = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    Driver_Controller.SwerveCommandXValue = 0.0;
                    Driver_Controller.SwerveCommandYValue = 0.0;
                    Driver_Controller.SwerveControlSet(true);
                    ++autoState;
                }
                break;
            case 7:
                shootAuto(true);
                break;
        }
    }

    public static void depotAuto(){
        ++cnt;
        Double depotA = ((alliance == 'R')?85.0:-85.0);
        switch(autoState){
            case 0:
                if (cnt >= 50*shootTimeSec){ // 5 seconds
                    Double depotX = ((alliance == 'R')?11.915394+4.62534-1.0:1.0),
                    depotY = 4.034663*((alliance == 'R')?0.25:1.75);
                    AutoDrive.setSpline(depotX, depotY, -2*((alliance == 'R')?1.0:-1.0)*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                Driver_Controller.SwerveCommandEncoderValue = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                shootAuto(true);
                break;
            case 1:
                Transport.setIntake(0.6);
                shootAuto(false);
                if (AutoDrive.driveSpline(depotA)){
                    Double depotX = ((alliance == 'R')?11.915394+4.62534-0.5:0.5),
                        depotY = 4.034663*((alliance == 'R')?0.35:1.65);
                    AutoDrive.setSpline(depotX, depotY, 0.0, ((alliance == 'R')?-1.0:1.0), 1.5, 50);
                    ++autoState;
                }
                break;
            case 2:
                Transport.setIntake(0.6);
                shootAuto(false);
                if (AutoDrive.driveSpline(depotA)){
                    Double depotX = ((alliance == 'R')?11.915394+4.62534-0.5:0.5),
                        depotY = 4.034663*((alliance == 'R')?0.75:1.25);
                    AutoDrive.setSpline(depotX, depotY, 0.0, ((alliance == 'R')?-1.0:1.0), 1.0, 50);
                    ++autoState;
                }
                break;
            case 3:
                Transport.setIntake(0.6);
                shootAuto(false);
                if (AutoDrive.driveSpline(depotA)){
                    Double desiredX = ((alliance == 'R')?
                        Turret.redTargetX[2]+1.538:
                        Turret.blueTargetX[2]-1.538),
                    desiredY = Turret.TargetY[2] +
                        ((alliance == 'R')?- 1.336606:1.336606);
                    AutoDrive.setSpline(desiredX, desiredY, 0.0, 0.0, speed/2, 50);
                    ++autoState;
                }
                break;
            case 4:
                Transport.setIntake(0.6);
                shootAuto(false);
                if (AutoDrive.driveSpline(depotA)){
                    Transport.setIntake(0.0);
                    ready = false;
                    ++autoState;
                }
                break;
            case 5:
                Driver_Controller.SwerveCommandEncoderValue = ((alliance == 'R')?-130.0:50.0);
                Driver_Controller.SwerveCommandXValue = 0.0;
                Driver_Controller.SwerveCommandYValue = 0.0;
                Driver_Controller.SwerveControlSet(true);
                shootAuto(true);
                break;
        }
    }
    public static int cnt = 0;
    public static void halfIntakeReturnAuto(){
        ++cnt;
        switch(autoState){
            case 0:
                if (cnt >= 50*shootTimeSec){
                    startIntakeAngle = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    enterNeutralPoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                shootAuto(true);
                break;
            case 1:
                if (shouldSkipMove || AutoDrive.driveSpline(startIntakeAngle)){
                    Transport.setIntake(0.6);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                shootAuto(false);
                break;
            case 2:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    Double yPosition = 4.034663+((destPoints[2] > 4.034663)?0.5:-0.5);
                    AutoDrive.setSpline((14.552041+1.988947)/2+((alliance == 'R')?0.5:-0.5), yPosition, 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 3:
                shootAuto(false);
                if (AutoDrive.driveSpline()){
                    enterAlliancePoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], 1.5*(destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 4:
                shootAuto(false);
                if (AutoDrive.driveSpline()){
                    Transport.setIntake(0.0);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 5:
                shootAuto(false);
                if (AutoDrive.driveSpline()){
                    Double desiredX = ((destPoints[0] > (14.552041+1.988947)/2)?
                        Turret.redTargetX[2]+1.2:
                        Turret.blueTargetX[2]-1.2),
                    desiredY = Turret.TargetY[2] +
                        ((destPoints[2] > 4.034663)?1.2:-1.2);
                    AutoDrive.setSpline(desiredX, desiredY, 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 6:
                shootAuto(false);
                if (AutoDrive.driveSpline((alliance == 'R')?0.0:180.0)){
                    Driver_Controller.SwerveCommandEncoderValue = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    Driver_Controller.SwerveCommandXValue = 0.0;
                    Driver_Controller.SwerveCommandYValue = 0.0;
                    Driver_Controller.SwerveControlSet(true);
                    ++autoState;
                }
                break;
            case 7:
                Integer time = Math.toIntExact(Math.round(DriverStation.getMatchTime()));
                System.out.println(time);
                if (cnt >= 15*50){
                    Turret.shooter1.set(0.0);
                    Turret.shooter2.set(0.0);
                    Transport.setTransport(0.0);
                    Transport.setSpindexer(0.0);
                    Transport.agitate(false);
                    enterNeutralPoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                    break;
                }
                shootAuto(true);
                break;
            case 8:
                if (shouldSkipMove || AutoDrive.driveSpline(((alliance == 'R')?180.0:0.0))){
                    Transport.setIntake(0.6);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 9:
                if (AutoDrive.driveSpline()){
                    Double yPosition = 4.034663+((destPoints[2] > 4.034663)?0.5:-0.5);
                    AutoDrive.setSpline((14.552041+1.988947)/2, yPosition, 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 10:
                if (AutoDrive.driveSpline()){
                    Driver_Controller.SwerveCommandEncoderValue = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    Driver_Controller.SwerveCommandXValue = 0.0;
                    Driver_Controller.SwerveCommandYValue = 0.0;
                    Driver_Controller.SwerveControlSet(true);
                    ++autoState;
                }
                break;
        }
    }

    // public static void DEFENSEAUTOYAYAYAYAYAYAY(){
    //     Double odoy = RobotContainer.drivetrain.getState().Pose.getY();
    //     Double odox = RobotContainer.drivetrain.getState().Pose.getX();
    //     Double defenseSpeedMpS = 0.1;
    //     switch(autoState){
    //         case 0:
    //             enterNeutralPoints();
    //             AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
    //             ++autoState;
    //             break;
    //         case 1:
    //             if (AutoDrive.driveSpline()){
    //                 AutoDrive.setSpline((14.552041+1.988947)/2, destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult*2, 0.0, speed, 50);
    //                 ++autoState;
    //             }
    //             break;
    //         case 2:
    //             if (AutoDrive.driveSpline()){
    //                 ++autoState;
    //             }
    //         case 3:
    //             if (destPoints[2] < 4.034663){
    //                 if (odoy > 4.034663*1.5){
    //                     ++autoState;
    //                     break;
    //                 }
    //                 Driver_Controller.SwerveCommandEncoderValue = 90;
    //                 Driver_Controller.SwerveCommandXValue = 0.0;
    //                 Driver_Controller.SwerveCommandYValue = defenseSpeedMpS;
    //                 break;
    //             }else{
    //                 if (odoy < 4.034663*0.5){
    //                     ++autoState;
    //                     break;
    //                 }
    //                 Driver_Controller.SwerveCommandEncoderValue = 90;
    //                 Driver_Controller.SwerveCommandXValue = 0.0;
    //                 Driver_Controller.SwerveCommandYValue = -defenseSpeedMpS;
    //                 break;
    //             }
    //         case 4:
                
                
    //     }
    // }

    public static Double startIntakeAngle = 0.0;
    public static void hubIntakeAuto(){
        ++cnt;
        switch(autoState){
            case 0:
                if (cnt >= 50*shootTimeSec){
                    enterNeutralPoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    startIntakeAngle = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    ++autoState;
                }
                shootAuto(true);
                break;
            case 1:
                if (shouldSkipMove || AutoDrive.driveSpline(startIntakeAngle)){
                    Transport.setIntake(0.6);
                    AutoDrive.setSpline(destPoints[0]*0.125+destPoints[1]*0.875, destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult/2, 0.0, speed, 50);
                    ++autoState;
                }
                shootAuto(false);
                break;
            case 2:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    AutoDrive.setSpline(destPoints[0]*0.125+destPoints[1]*0.875, 4.034663*2-destPoints[2]*0.9, 0.0, -(4.034663-destPoints[2])/2*endVeloMult, speed, 50);
                    ++autoState;
                }
                break;
            case 3:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    enterAlliancePoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], 1.5*(destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 4:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    Transport.setIntake(0.0);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 5:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    Driver_Controller.SwerveCommandEncoderValue = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    Driver_Controller.SwerveCommandXValue = 0.0;
                    Driver_Controller.SwerveCommandYValue = 0.0;
                    Driver_Controller.SwerveControlSet(true);
                    ++autoState;
                }
                break;
            case 6:
                // if (cnt >= 15*50){
                //     Turret.shooter1.set(0.0);
                //     Turret.shooter2.set(0.0);
                //     Transport.setTransport(0.0);
                //     Transport.setSpindexer(0.0);
                //     Transport.agitate(false);
                //     enterNeutralPoints();
                //     AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                //     ++autoState;
                //     break;
                // }
                shootAuto(true);
                break;
            // case 8:
            //     if (shouldSkipMove || AutoDrive.driveSpline(((alliance == 'R')?180.0:0.0))){
            //         Transport.setIntake(0.6);
            //         AutoDrive.setSpline(destPoints[1], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
            //         ++autoState;
            //     }
            //     break;
            // case 9:
            //     if (AutoDrive.driveSpline()){
            //         Double yPosition = 4.034663+((destPoints[2] > 4.034663)?0.5:-0.5);
            //         AutoDrive.setSpline((14.552041+1.988947)/2, yPosition, 0.0, 0.0, speed, 50);
            //         ++autoState;
            //     }
            //     break;
            // case 10:
            //     if (AutoDrive.driveSpline()){
            //         Driver_Controller.SwerveCommandEncoderValue = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
            //         Driver_Controller.SwerveCommandXValue = 0.0;
            //         Driver_Controller.SwerveCommandYValue = 0.0;
            //         Driver_Controller.SwerveControlSet(true);
            //         ++autoState;
            //     }
            //     break;
        }
    }

    public static void reset(){
        endVeloMult = 0.3*speed;
        revdIntakeCnt = 0;
        cnt = 0;
        Turret.encoderStatus = 's';
        bigSpeed = 0.0;
        autoState = 0;
        alliance = DriverStation.getAlliance().toString().charAt(9);
        ready = false;
    }
}