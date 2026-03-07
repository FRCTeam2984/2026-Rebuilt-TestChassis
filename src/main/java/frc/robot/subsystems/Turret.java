package frc.robot.subsystems;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.concurrent.CopyOnWriteArrayList;

import org.ejml.dense.row.decompose.UtilDecompositons_CDRM;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.*;


public class Turret {
    public static DigitalInput resettingSensor = new DigitalInput(Constants.turretSensorPort);
    public static char encoderStatus = 's'; // at first, we don't know if the encoder is correct
    public static Double maxPower = 0.5, maxError = 1.0/*TODO FIXME*/; // max power allowed to send and furthest the turret can be from ideal without sending any power
    public static Double[] blueTargetX = {1.988947, 1.988947, 4.62534}, TargetY = {2.0173315, 6.05119945, 4.034663}, redTargetX = {14.552041, 14.552041, 11.915394}; // coordinates for targets on the field
    public static Double turretOffX = 0.23, // offset of the turret in meters from the center of the robot, forward is positive
                         turretOffY = 0.0; // left is positive
    public static Double spinRatio = 32.0;
    public static Double turret_zero = 0.0;

    public static Double distance = 0.0, modTurretOff = 0.0; // current distance from the target point, error in target angle
    public static Servo servo1 = new Servo(Constants.servo1ID);
    public static Servo servo2 = new Servo(Constants.servo2ID);
    public static Servo servoInverted = new Servo(Constants.servoInvertedID);
    public static SparkMax turretSpin = new SparkMax(Constants.turretSpinID, MotorType.kBrushless);
    public static RelativeEncoder turretEncoder = turretSpin.getEncoder();
    public static TalonFX shooter1 = new TalonFX(Constants.shooterID1),
                          shooter2 = new TalonFX(Constants.shooterID2);

    public static ArrayList<ArrayList<Double[]>> cowlInterpolation = new ArrayList<>(0);

    public static Boolean spinMotorWorking = false;
    public static void initMotors(){
        try{
            turretSpin = new SparkMax(Constants.turretSpinID, MotorType.kBrushless);
            turretEncoder = turretSpin.getEncoder();
            spinMotorWorking = true;
        }catch(Exception e){
            System.out.println("no transport motor");
        }
    }

    public static Double desiredSpeed = 0.0, cowlAngle = 0.5, interpolatedTurretOffset = 0.0, targetAngle = 0.0;
    public static void calcDist(){
        // grab odometry values
        Double angle = Math.atan2(RobotContainer.drivetrain.getState().Speeds.vyMetersPerSecond,
            RobotContainer.drivetrain.getState().Speeds.vxMetersPerSecond)+
            Math.toRadians(RobotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble());
        Double magnitude = Math.sqrt(Math.pow(RobotContainer.drivetrain.getState().Speeds.vyMetersPerSecond, 2)
            +Math.pow(RobotContainer.drivetrain.getState().Speeds.vxMetersPerSecond, 2));
        Double xVelo = -magnitude*Math.sin(angle)*0.5;// 1/2 second travel time
        Double yVelo = -magnitude*Math.cos(angle)*0.5;
        //System.out.printf("xVelo = %.3f, yVelo = %.3f\n", xVelo, yVelo);

        Double odoY = RobotContainer.drivetrain.getState().Pose.getY()+yVelo;
        Double odoX = RobotContainer.drivetrain.getState().Pose.getX()+xVelo;
        Double odoA = Math.toRadians(((RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees()+ 360*1000 + 180)%360)+RobotContainer.drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()/10);

        Double destX, destY; // find where our destination is
        if (Robot.alliance == 'B'){// if alliance is blue
            if (odoX < blueTargetX[2]){ // if we are in blue alliance zone
                destX = blueTargetX[2];
                destY = TargetY[2];
            }else{ // in neutral zone/red alliance zone
                destX = blueTargetX[0];
                destY = TargetY[((odoY > TargetY[2])?1:0)]; // make it the lower/bigger y value based on if cur y value is above or below hub value
            }
        }else{ // if alliance is red
            if (odoX > redTargetX[2]){ // if we are in red alliance zone
                destX = redTargetX[2];
                destY = TargetY[2];
            }else{ // in neutral zone/blue alliance zone
                destX = redTargetX[0];
                destY = TargetY[((odoY > TargetY[2])?1:0)]; // make it the lower/bigger y value based on if cur y value is above or below hub value
            }
        }
        
        Double turretAngle = (turretEncoder.getPosition()-turret_zero)*360/spinRatio-5;
        Double turrX, turrY; // variables that represent position of the TURRET relative to the target location
        //turretAngle += Driver_Controller.offsetSlider();

        turrX = odoX + turretOffX*Math.cos(odoA) - turretOffY*Math.sin(odoA) - destX;
        turrY = odoY + turretOffX*Math.sin(odoA) + turretOffY*Math.cos(odoA) - destY;
        // calculate above by odometry value and then trigonometric ratios using the robot angle to find turret pos on the field, then subtract destination values

        distance = Math.sqrt(Math.pow(turrX , 2) + Math.pow(turrY, 2)); // pythagorean theorum
        targetAngle = (15380*360-Math.toDegrees(Math.atan2(turrX, turrY)+odoA)+((Driver_Controller.flipDrive())?180.0:0.0)-180)%360-180;
        Double turretOffset = targetAngle - turretAngle;
        if ((Robot.alliance == 'B' && odoX > blueTargetX[2]) || (Robot.alliance == 'R' && odoX < redTargetX[2])){
            interpolatedTurretOffset = ((Driver_Controller.useOffsets())?Driver_Controller.offsetSlider():0.0);
            desiredSpeed = 50.0;
            cowlAngle = 0.0;
            return;
        }

        //distance += 1.0;

        // interpolate for the speed, cowl angle, and turret offset
        ArrayList<Double[]> upper, lower;
        int currentUpper = -1;
        while ((upper = cowlInterpolation.get(++currentUpper)).get(0)[0] < targetAngle){}
        lower = cowlInterpolation.get(Math.max(0, currentUpper-1));

        Double[] Ldist1, Udist1;
        currentUpper = 1;
        while (upper.get(currentUpper)[0] < distance){currentUpper++;}
        Udist1 = lower.get(currentUpper);
        Ldist1 = upper.get(Math.max(1, currentUpper-1));

        Double[] Ldist2, Udist2;
        currentUpper = 1;
        while (lower.get(currentUpper)[0] < distance){currentUpper++;}
        Udist2 = lower.get(currentUpper);
        Ldist2 = lower.get(Math.max(1, currentUpper-1));

        Double[] interpolated = new Double[3];
        for (int i = 1; i <= 3; ++i){
            Double temp1 = Udist1[i] - (Udist1[0]-distance)/(Udist1[0]-Ldist1[0])*(Udist1[i]-Ldist1[i]);
            Double temp2 = Udist2[i] - (Udist2[0]-distance)/(Udist2[0]-Ldist2[0])*(Udist2[i]-Ldist2[i]);
            interpolated[i-1] = temp2 - (upper.get(0)[0]-targetAngle)/(upper.get(0)[0]-lower.get(0)[0])*(temp2-temp1);
        }
        if (!Driver_Controller.manualSwitch())interpolatedTurretOffset = interpolated[2];
        if (Driver_Controller.useOffsets()) interpolatedTurretOffset += Driver_Controller.offsetSlider();
        desiredSpeed = interpolated[1];
        if (Driver_Controller.useOffsets()) desiredSpeed += Driver_Controller.shooterOffsetSlider();
        cowlAngle = interpolated[0];
        desiredSpeed = Math.max(10.0, Math.min(60.0, desiredSpeed));

        modTurretOff = 180 - (turretOffset + interpolatedTurretOffset + 180 + 360*Math.pow(10, 5))%360;//in the range of -180 to 180 degrees
    }

    public static Double spinTurret(){
        if (Driver_Controller.pauseTurret())return 0.0;
        Double maxPowDist = 20.0; // TODO FIXME proportional power to error, at 20 degrees off, it will be going at maxPower
        Double power = modTurretOff/maxPowDist*maxPower;

        if (Math.abs(modTurretOff) < maxError){ // if really close to perfect, then no power needed
            return 0.0;
        }
        
        Double minSpeed = 600.0;//TODO FIXME rpm
	    //if (Math.abs(turretEncoder.getVelocity()) < minSpeed){ // if the motor is slow, make it faster
            Double maxAdjust = Math.min(1.0, Math.max(-1.0, modTurretOff/maxPowDist)); // the desired max speed changes based on our distance
            Double speedCompensation = 0.1*maxAdjust*(minSpeed-Math.abs(turretEncoder.getVelocity()))/minSpeed;//TODO FIXME maybe change constants
            power += speedCompensation;
        //}
        power = Math.min(maxPower, Math.max(-maxPower, power)); // limit power to between -maxPower and +maxPower	
        return -power/4;// TODO FIXME maybe make negative
    }
    
    public static Double resetEncoder(){
        Double fastSpeed = 0.5, slowSpeed = -0.05;
        switch(encoderStatus){
            case 's': // first state, go fast until we trigger the sensor
                if (!resettingSensor.get()){
                    encoderStatus = 'p';
                }
                return fastSpeed;
            case 'p': // we triggered the beam break sensor and are waiting to pass it
                if (!resettingSensor.get() == false){
                    encoderStatus = 'c';
                    return slowSpeed;
                }
                return fastSpeed;
            case 'c': // we are close to the beam break sensor, go slower until we trigger again
                if (!resettingSensor.get()){
                    turretEncoder.setPosition(0.0);
                    turret_zero = turretEncoder.getPosition();
                    encoderStatus = 'g'; // we are good
                    return 0.0;
                }
                return slowSpeed;
            default:
                return 0.0;
        }
    }

    public static Boolean readFiles(){ // function to read the interpolation file
        try{
            FileReader fileReader = new FileReader("/home/admin/cowlInterpolation.txt");
            BufferedReader bufferedReader = new BufferedReader(fileReader);
            String line = null;
            int curSubarray = -1;
            Integer numberValues;
            Double[] newArray;
            while ((line = bufferedReader.readLine()) != null) {
                switch(line){
                    case "{": // add a new subarray
                        ++curSubarray;
                        ArrayList<Double[]> newAdd = new ArrayList<>(0);
                        cowlInterpolation.add(newAdd);
                        break;
                    case "}":
                        Double[] lastArray = cowlInterpolation.get(curSubarray).get(cowlInterpolation.get(curSubarray).size()-1);
                        numberValues = lastArray.length;
                        newArray = new Double[numberValues];
                        for (int i = 1; i < numberValues; ++i){
                            newArray[i] = lastArray[i];
                        }
                        newArray[0] = 99999999.0;
                        cowlInterpolation.get(curSubarray).add(newArray);
                        break;
                    default:
                        String[] words = line.trim().split("\\s+");
                        numberValues = words.length;
                        Double[] converted = new Double[numberValues];
                        for (int i = 0; i < numberValues; ++i){
                            converted[i] = Double.parseDouble(words[i]);
                        }
                        if (cowlInterpolation.get(curSubarray).size() == 1){
                            newArray = new Double[numberValues];
                            for (int i = 1; i < numberValues; ++i){
                                newArray[i] = converted[i];
                            }
                            newArray[0] = -99999999.0;
                            cowlInterpolation.get(curSubarray).add(newArray);
                        }
                        cowlInterpolation.get(curSubarray).add(converted);
                        break;
                }
            }
            bufferedReader.close();
            return true;
        }catch(FileNotFoundException e){
            System.out.println("uh oh - failed to read from interpolaton file - FileNotFoundException");
        }catch(IOException e){
            System.out.println("uh oh - failed to read from interpolaton file - IOException");
        }
        return false;
    }

    public static Double[] curPower = {0.0, 0.0};
    public static Double[] prevSpeed1 = {0.0, 0.0, 0.0, 0.0, 0.0},
                           prevSpeed2 = {0.0, 0.0, 0.0, 0.0, 0.0};
    public static int speedIndex = 0;

    public static Boolean close = false;

    public static Double[] speedController(){
        close = true;
        int nextSpeedIndex = (speedIndex+1)%5;

        Double velocity = Math.abs(shooter1.getVelocity().getValueAsDouble()); // rotations per second
        prevSpeed1[speedIndex] = velocity;
        Double accel = (velocity-prevSpeed1[nextSpeedIndex]);// rotations per second per second, averaged over .1 second
        //if (speedIndex == 0) System.out.print(velocity + ",  ");

        Double p, i, d;
        p = desiredSpeed*0.01;
        if ((desiredSpeed-velocity) < 0){
            p += (desiredSpeed-velocity)*0.005;
        }else{
            p += (desiredSpeed-velocity)*0.002;
        }
        i = (desiredSpeed-velocity)*0.000025;
        curPower[0] += i;
        d = accel*(-0.015);
        Double power1 = Math.min(1.0, Math.max(0.0, p+curPower[0]+d));

        if (velocity < desiredSpeed*0.95 || velocity > desiredSpeed*1.1)
            close = false;

        velocity = Math.abs(shooter2.getVelocity().getValueAsDouble()); // rotations per second
        prevSpeed2[speedIndex] = velocity;
        accel = (velocity-prevSpeed2[nextSpeedIndex]);// rotations per second per second, averaged over .1 second
        //if (speedIndex == 0) System.out.println(velocity);

        p = desiredSpeed*0.01;
        if ((desiredSpeed-velocity) < 0){
            p += (desiredSpeed-velocity)*0.005;
        }else{
            p += (desiredSpeed-velocity)*0.002;
        }
        i = (desiredSpeed-velocity)*0.000025;
        curPower[1] += i;
        d = accel*(-0.015);
        Double power2 = Math.min(1.0, Math.max(0.0, p+curPower[1]+d));

        if (velocity < desiredSpeed*0.95 || velocity > desiredSpeed*1.1)
            close = false;
        
        if (close){
            LED.setPattern('B');
        }else{
            LED.setPattern('R');
        }
        speedIndex = nextSpeedIndex;
        return new Double[] {power1, power2};
    }
}
//fix lines that say TODO FIXME for arbitrary constants