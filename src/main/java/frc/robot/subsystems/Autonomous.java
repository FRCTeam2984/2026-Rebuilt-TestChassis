package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.RobotContainer;

// this is mostly organization for our different autos
public class Autonomous {
    public static char alliance;
    public static Boolean ready, idleShooter = true, shootFirst = false;
    public static Integer revdIntakeCnt = 0;
    public static void shootAuto(Boolean transport){ // this function automatically shoots
        if (revdIntakeCnt <= 25){
            Transport.setIntake(-0.4);
            if (revdIntakeCnt == 25){
                Transport.setIntake(0.0);
            }
            ++revdIntakeCnt;
        }
        Double shooterPower = Turret.resetEncoder();
        //System.out.println(Turret.encoderStatus);
        if (Math.abs(shooterPower) < 0.0001){
            shooterPower = Turret.spinTurret();
        }
        Turret.turretSpin.set(shooterPower);
        if ((idleShooter == false) && (transport == false)){
            ready = false;
            Turret.shooter1.set(0.0);
            Turret.shooter2.set(0.0);
            Transport.setTransport(0.0);
            Transport.agitate(false);
            return;
        }
        // spin up shooter and reset/aim turret and stuff
        Turret.calcDist();
        Double[] power = Turret.speedController();
        Turret.shooter1.set(power[0]);
        Turret.shooter2.set(-power[1]);

        // checks everything if ready to shoot
        ready = Turret.close
            && (Math.abs(Turret.modTurretOff) < 5);
        if (ready && transport){
            long time = System.nanoTime()%(2000*1000*1000);
            if (time < 500*1000*1000){
                Transport.setTransport(0.65);
            }else{
                Transport.setTransport(-0.65);
            }
            Transport.agitate(true);
        }
    }
    public static void shootAuto(){
        shootAuto(true);
    }

    public static Double[] destPoints = {0.0, 0.0, 0.0}; // x1, x2, y
    public static void enterNeutralPoints(){
        Double odoy = RobotContainer.drivetrain.getState().Pose.getY();
        Double odox = RobotContainer.drivetrain.getState().Pose.getX();
        if (odoy > 4.034663){
            destPoints[2] = 7.4247756+0.1;
        }else{
            destPoints[2] = (4.034663*2-7.4247756)-0.1;
        }
        if (odox > (11.915394+4.62534)/2){
            destPoints[0] = 11.915394;
            if (odox < (11.915394+0.5)) destPoints[0] -= 1.0;
            destPoints[1] = 11.915394-2;
        }else{
            destPoints[0] = 4.62534;
            if (odox > (4.62534-0.5)) destPoints[0] += 1.0;
            destPoints[1] = 4.62534+2;
        }
    }

    public static void enterAlliancePoints(){
        Double odoy = RobotContainer.drivetrain.getState().Pose.getY();
        if (odoy > 4.034663){
            destPoints[2] = 7.4247756+0.1;
        }else{
            destPoints[2] = (4.034663*2-7.4247756)-0.1;
        }
        
        if (alliance == 'R'){
            destPoints[0] = 11.915394;
            destPoints[1] = 11.915394+2;
        }else{
            destPoints[0] = 4.62534;
            destPoints[1] = 4.62534-2;
        }
    }

    public static final Double speed = 1.0, endVeloMult = 0.3;
    public static Integer autoState = 0;
    public static void shuttleAuto(){
        //shootAuto();
        Double driveAngle = (alliance == 'R')?80.0:-100.0;
        switch(autoState){
            case 0:
                enterNeutralPoints();
                AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                ++autoState;
            case 1:
                if (AutoDrive.driveSpline()){
                    AutoDrive.setSpline(destPoints[1], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                    Transport.setIntake(0.4);
                }
                break;
            case 2:
                if (AutoDrive.driveSpline()){
                    AutoDrive.setSpline((14.552041+1.988947)/2, 4.034663-(destPoints[2]-4.034663)*0.6, 0.0, (destPoints[2]-4.034663)/2*endVeloMult, speed, 50);
                    ++autoState;
                }
                break;
            case 3:
                if (AutoDrive.driveSpline(-driveAngle)){
                    Double xAdd = (alliance == 'R')?0.5:-0.5;
                    AutoDrive.setSpline((14.552041+1.988947)/2+xAdd, 4.034663+(destPoints[2]-4.034663)*0.6, 0.0, -(destPoints[2]-4.034663)/2*endVeloMult, speed, 50);
                    ++autoState;
                }
                break;
            case 4:
                if (AutoDrive.driveSpline(driveAngle)){
                    Double xAdd = (alliance == 'R')?1.0:-1.0;
                    AutoDrive.setSpline((14.552041+1.988947)/2+xAdd, 4.034663-(destPoints[2]-4.034663)*0.6, 0.0, (destPoints[2]-4.034663)/2*endVeloMult, speed, 50);
                    ++autoState;
                }
                break;
            case 5:
                if (AutoDrive.driveSpline(-driveAngle)){
                    ++autoState;
                }
                break;
            case 6:
                Transport.setIntake(0.0);
                Driver_Controller.SwerveControlSet(false);
                break;
        }
    }

    public static int shootCnt = 0;
    public static void outpostAuto(){
        Double outpostA = ((alliance == 'B')?0.0:180);
        switch(autoState){
            case 0:
                shootAuto(true);
                if (ready) ++shootCnt;
                if (shootCnt >= 50*5){ // 5 seconds
                    Double outpostX = ((alliance == 'R')?11.915394+4.62534-0.5:0.5),
                    outpostY = ((alliance == 'R')?(4.034663*2-0.5):0.5);
                    AutoDrive.setSpline(outpostX, outpostY, 0.0, 0.0, speed, 50);
                    shootCnt = 0;
                    ++autoState;
                }
                break;
            case 1:
                shootAuto(false);
                if (AutoDrive.driveSpline(outpostA)){
                    ++autoState;
                }
                break;
            case 2:
                shootAuto(false);
                Driver_Controller.SwerveControlSet(false);
                ++shootCnt;
                if (shootCnt >= 50*2){ // 2 seconds wait for fuel from outpost
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
                Driver_Controller.SwerveControlSet(false);
                shootAuto(true);
                break;
        }
    }

    public static void intakeAuto(){
        Double driveAngle = ((destPoints[2] < 4.034663)?-1:1)*((alliance == 'R')?80.0:-100.0);
        switch(autoState){
            case 0:
                if (shootFirst == false || shootCnt >= 50*5){
                    enterNeutralPoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    shootCnt = 0;
                    ++autoState;
                }
                shootAuto(true);
                if (ready) ++shootCnt;
                break;
            case 1:
                shootAuto(false);
                if (AutoDrive.driveSpline()){
                    Transport.setIntake(0.4);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 2:
                shootAuto(false);
                if (AutoDrive.driveSpline()){
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
                    ++autoState;
                }
                break;
            case 6:
                shootAuto(false);
                if (AutoDrive.driveSpline()){
                    ++autoState;
                }
                break;
            case 7:
                shootAuto(true);
                Driver_Controller.SwerveControlSet(false);
                break;
        }
    }
    
    public static void halfIntakeAuto(){
        switch(autoState){
            case 0:
                if (shootFirst == false || shootCnt >= 50*5){
                    enterNeutralPoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    shootCnt = 0;
                    ++autoState;
                }
                shootAuto(true);
                if (ready) ++shootCnt;
                break;
            case 1:
                shootAuto(false);
                if (AutoDrive.driveSpline()){
                    Transport.setIntake(0.4);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 2:
                shootAuto(false);
                if (AutoDrive.driveSpline()){
                    Double yPosition = 4.034663+((destPoints[2] > 4.034663)?0.5:-0.5);
                    AutoDrive.setSpline((14.552041+1.988947)/2, yPosition, 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 3:
                shootAuto(false);
                if (AutoDrive.driveSpline()){
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
                    AutoDrive.setSpline(desiredX, desiredY, 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 6:
                shootAuto(false);
                if (AutoDrive.driveSpline()){
                    ++autoState;
                }
                break;
            case 7:
                shootAuto(true);
                break;
        }
    }

    public static void reset(){
        Turret.encoderStatus = 's';
        autoState = 0;
        shootCnt = 0;
        alliance = DriverStation.getAlliance().toString().charAt(9);
    }
}
