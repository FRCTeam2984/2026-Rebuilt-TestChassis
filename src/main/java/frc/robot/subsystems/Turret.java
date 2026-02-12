package frc.robot.subsystems;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.FileReader;
import java.util.ArrayList;

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
    public static Double turretOffX = 0.1, // offset of the turret in meters from the center of the robot, forward is positive
                         turretOffY = 0.0; // left is positive
    public static Double spinRatio = 160.0;

    public static Double distance = 0.0, modTurretOff = 0.0; // current distance from the target point, error in target angle
    public static Servo servo1 = new Servo(Constants.servo1ID);
    public static Servo servo2 = new Servo(Constants.servo2ID);
    public static Servo servoInverted = new Servo(Constants.servoInvertedID);
    public static SparkMax turretSpin = new SparkMax(Constants.turretSpinID, MotorType.kBrushless);
    public static RelativeEncoder turretEncoder = turretSpin.getEncoder();
    public static TalonFX shooter1 = new TalonFX(Constants.shooterID1),
                          shooter2 = new TalonFX(Constants.shooterID2);

    public static ArrayList<ArrayList<Double>> cowlInterpolation = new ArrayList<>(0);

    public static void calcDist(){
        // grab odometry values
        Double odoY = RobotContainer.drivetrain.getState().Pose.getY();
        Double odoX = RobotContainer.drivetrain.getState().Pose.getX();
        Double odoA = Math.toRadians(RobotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble());

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
        
        Double turretAngle = turretEncoder.getPosition()*360/spinRatio;
        Double turrX, turrY; // variables that represent position of the TURRET relative to the target location

        turrX = odoX + turretOffX*Math.cos(odoA) - turretOffY*Math.sin(odoA) - destX;
        turrY = odoY + turretOffX*Math.sin(odoA) + turretOffY*Math.cos(odoA) - destY;
        // calculate above by odometry value and then trigonometric ratios using the robot angle to find turret pos on the field, then subtract destination values

        distance = Math.sqrt(Math.pow(turrX , 2) + Math.pow(turrY, 2)); // pythagorean theorum
        Double targetAngle = Math.toDegrees(Math.atan2(turrX, turrY)+odoA);
        Double turretOffset = targetAngle - turretAngle;
        modTurretOff = 180 - (turretOffset + 180 + 360*Math.pow(10, 5))%360;//in the range of -180 to 180 degrees
    }

    public static Double spinTurret(){
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
        return -power;// TODO FIXME maybe make negative
    }
    
    public static void resetEncoder(){
        Double fastSpeed = maxPower, slowSpeed = -0.1;
        switch(encoderStatus){
            case 's': // first state, go fast until we trigger the sensor
                if (resettingSensor.get()){
                    encoderStatus = 'p';
                }
                turretSpin.set(fastSpeed);
                break;
            case 'p': // we triggered the beam break sensor and are waiting to pass it
                if (resettingSensor.get() == false){
                    encoderStatus = 'c';
                    turretSpin.set(slowSpeed);
                }else{
                    turretSpin.set(fastSpeed);
                }
                break;
            case 'c': // we are close to the beam break sensor, go slower until we trigger again
                if (resettingSensor.get()){
                    turretEncoder.setPosition(0.0);
                    encoderStatus = 'g'; // we are good
                    turretSpin.set(0.0);
                }else{
                    turretSpin.set(slowSpeed);
                }
                break;
            case 'g':
                return;
        }
    }

    public static Boolean readFiles(){ // function to read the interpolation file
        try{
            FileReader fileReader = new FileReader("C:\\Users\\team2984\\Documents\\GitHub\\2026-Rebuilt-TestChassis\\src\\main\\java\\frc\\robot\\cowlInterpolation.txt");
            BufferedReader bufferedReader = new BufferedReader(fileReader);
            String line = null;
            int curSubarray = -1;
            while ((line = bufferedReader.readLine()) != null) {
                switch(line){
                    case "{": // add a new subarray
                        ++curSubarray;
                        ArrayList<Double> newAdd = new ArrayList<>(0);
                        newAdd.add(-Math.pow(10, 6));
                        cowlInterpolation.add(newAdd);
                        break;
                    case "}":
                        break;
                    default:
                        int size = line.length();
                        if (line.charAt(size-1) != ','){ // return if the last character is not ,
                            break;
                        }
                        cowlInterpolation.get(curSubarray).add( // add the value to the array
                            Double.parseDouble(
                            line.substring(0, size-1)));
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

    public static Double moveCowl(){
        try{
            // interpolate
            Double lowerInp = cowlInterpolation.get(0).get(0), upperInp = cowlInterpolation.get(0).get(1); // will be the input values above and below the current distance
            int i = 0, inputSize = cowlInterpolation.get(0).size();
            while (++i < inputSize){ // go through the inputs until it surpasses the top value or upperInp >= our input distance value
                lowerInp = cowlInterpolation.get(0).get(i-1);
                if ((upperInp = cowlInterpolation.get(0).get(i)) >= distance){
                    break;
                }
            }
            if (i == inputSize){ // if the value is extreme
                return -1.0;
            }
            Double dist1 = cowlInterpolation.get(1).get(i-1), // corresponding angle outputs to lowerInp and upperInp
                   dist2 = cowlInterpolation.get(1).get(i);
            return dist2 -
                -(upperInp-distance)/(upperInp-lowerInp)   //ratio of how far the input is from val2 versus val1 is to val2
                *(dist2-dist1);    //the distance between the outputs of val1 and val2
        }catch(IndexOutOfBoundsException e){
            return -1.0; 
        }
    }

    public static Double[] curPower = {0.0, 0.0};

    public static Double[] speedController(){
        Double desiredSpeed = 70.0;//-1.0;
        /*try{
            // interpolate
            Double lowerInp = cowlInterpolation.get(0).get(0), upperInp = cowlInterpolation.get(0).get(1); // will be the input values above and below the current distance
            int i = 0, inputSize = cowlInterpolation.get(0).size();
            while (++i < inputSize){ // go through the inputs until it surpasses the top value or upperInp >= our input distance value
                lowerInp = cowlInterpolation.get(0).get(i-1);
                if ((upperInp = cowlInterpolation.get(0).get(i)) >= distance){
                    break;
                }
            }
            if (i == inputSize){ // if the value is extreme
                return curPower = new Double[] {0.0, 0.0};
            }
            Double speed1 = cowlInterpolation.get(0).get(i-1), // corresponding angle outputs to lowerInp and upperInp
                   speed2 = cowlInterpolation.get(0).get(i);
            desiredSpeed = speed2 -
                -(upperInp-distance)/(upperInp-lowerInp)   //ratio of how far the input is from speed2 versus speed1 is to speed2
                *(speed2-speed1);    //the distance between the outputs of speed1 and speed2
        }catch(IndexOutOfBoundsException e){
            return curPower = new Double[] {0.0, 0.0};
        }*/
        Boolean close = true;

        Double velocity = Math.abs(shooter1.getVelocity().getValueAsDouble()); // rotations per second
        Double accel = shooter1.getAcceleration().getValueAsDouble(); // rotations per second per second
        System.out.print(velocity + ",   ");

        if (accel < (desiredSpeed-velocity)){
            curPower[0] += (desiredSpeed-velocity)/desiredSpeed/50; // increase/decrease power based on whether it is going to make it to the right speed based on acceleration
        }
        curPower[0] = Math.min(1.0, Math.max(0.0, curPower[0]));

        if (velocity < desiredSpeed*0.8 || velocity > desiredSpeed*1.2)
            close = false;

        velocity = Math.abs(shooter2.getVelocity().getValueAsDouble());
        accel = shooter2.getAcceleration().getValueAsDouble();
        System.out.println(velocity);

        if (accel < (desiredSpeed-velocity)){
            curPower[1] += (desiredSpeed-velocity)/desiredSpeed/50;
        }
        curPower[1] = Math.min(1.0, Math.max(0.0, curPower[1]));

        if (velocity < desiredSpeed*0.8 || velocity > desiredSpeed*1.2)
            close = false;
        
        if (close){
            LED.setPattern('R');
        }else{
            LED.setPattern('B');
        }

        System.out.println(curPower[0] + ", " + curPower[1]);
        return curPower;
    }
}
//fix lines that say TODO FIXME for arbitrary constants