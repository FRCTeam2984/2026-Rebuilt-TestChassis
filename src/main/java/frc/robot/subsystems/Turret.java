package frc.robot.subsystems;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class Turret {
    public static DigitalInput resettingSensor = new DigitalInput(Constants.turretSensorPort);
    public static char encoderStatus = 's'; // at first, we don't know if the encoder is correct
    public static Double maxPower = 0.5, maxError = 1.0; // max power allowed to send and furthest the turret can be from ideal without sending any power
    public static Double[] blueTargetX = {1.988947, 1.988947, 4.62534}, TargetY = {2.0173315, 6.05119945, 4.034663}, redTargetX = {14.552041, 14.552041, 11.915394}; // coordinates for targets on the field
    public static Double turretOffX = -0.23, // offset of the turret in meters from the center of the robot, forward is positive
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
        }catch(Exception e){}
    }
    public static Integer update_time;

    public static Double desiredSpeed = 0.0, cowlAngle = 0.5, interpolatedTurretOffset = 0.0, targetAngleDeg = 0.0;
    public static Boolean inAlliance = false;
    public static void calcDist(){
            // calculate the effective distance and angle to the hub while the robot is stationary OR moving/turning
            // outputs (all from interpolation table):
            //    desiredSpeed = shooter motor speed
            //    cowlAngle
            //    targetAngleDeg = robot turret forwards to hub angle
            //    interplatedTurretOffset = +/- offset to be added to turretAngle later
            //    distance = turret to hub after adding robot speed
            //    modTurretOff = error in turret angle
        try{
        // grab odometry values
        Double fieldRobotAngleDeg = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
        Double fieldRobotAngleRad = Math.toRadians(fieldRobotAngleDeg);
        Double robotVeloX = RobotContainer.drivetrain.getState().Speeds.vxMetersPerSecond;
        Double robotVeloY = RobotContainer.drivetrain.getState().Speeds.vyMetersPerSecond;
        Double fieldDriveVeloAngle = Math.atan2(robotVeloY, robotVeloX)+
            Math.toRadians(fieldRobotAngleDeg); // TODO, fixme: test for  both red and blue
        Double magnitude = Math.sqrt(Math.pow(robotVeloX, 2)+Math.pow(robotVeloY, 2));
        final Double flightTime = 1.10; // 1/2 second flight time of the fuel
        Double fieldVeloX = magnitude*Math.cos(fieldDriveVeloAngle)*flightTime;
        Double fieldVeloY = magnitude*Math.sin(fieldDriveVeloAngle)*flightTime;
        //System.out.printf("xVelo = %.3f, yVelo = %.3f\n", xVelo, yVelo);

        Double projectedOdoY = RobotContainer.drivetrain.getState().Pose.getY() + fieldVeloY;
        Double projectedOdoX = RobotContainer.drivetrain.getState().Pose.getX() + fieldVeloX;
        
        final Double turretLagTime = 0.15; // turret lags behind by x seconds: TODO, fixme: this should be an interpolation table!!!
        Double projectedOdoA = Math.toRadians(((
            fieldRobotAngleDeg+
            RobotContainer.drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()*turretLagTime+ // degreesPerSec * lag(s) = pre-aiming
            360*132300 + 180)%360));

        Double destX, destY; // find where our destination is
        if (Robot.alliance == 'B'){// if alliance is blue
            if (projectedOdoX < blueTargetX[2]){ // if we are in blue alliance zone
                destX = blueTargetX[2];
                destY = TargetY[2];
            }else{ // in neutral zone/red alliance zone
                LED.giveRange('d');
                destX = blueTargetX[0];
                destY = TargetY[((projectedOdoY > TargetY[2])?1:0)]; // make it the lower/bigger y value based on if cur y value is above or below hub value
            }
        }else{ // if alliance is red
            if (projectedOdoX > redTargetX[2]){ // if we are in red alliance zone
                destX = redTargetX[2];
                destY = TargetY[2];
            }else{ // in neutral zone/blue alliance zone
                LED.giveRange('d');
                destX = redTargetX[0];
                destY = TargetY[((projectedOdoY > TargetY[2])?1:0)]; // make it the lower/bigger y value based on if cur y value is above or below hub value
            }
        }
        
        Double turretAngleDeg = (turretEncoder.getPosition()-turret_zero)*360/spinRatio-5;
        Double turrTargetX, turrTargetY; // variables that represent position of the TURRET relative to the target location
        //turretAngle += Driver_Controller.offsetSlider();

        turrTargetX = projectedOdoX + turretOffX*Math.cos(fieldRobotAngleRad) - turretOffY*Math.sin(fieldRobotAngleRad) - destX;
        turrTargetY = projectedOdoY + turretOffX*Math.sin(fieldRobotAngleRad) + turretOffY*Math.cos(fieldRobotAngleRad) - destY;
        // calculate above by odometry value and then trigonometric ratios using the robot angle to find turret pos on the field, then subtract destination values

        distance = Math.sqrt(Math.pow(turrTargetX , 2) + Math.pow(turrTargetY, 2)); // pythagorean theorum
        Double prevAngle = targetAngleDeg;
        targetAngleDeg = (15380*360-Math.toDegrees(Math.atan2(turrTargetX, turrTargetY)+projectedOdoA)+((Driver_Controller.flipDrive())?180.0:0.0)-180)%360-180;
        Double turretErrorDeg = targetAngleDeg - turretAngleDeg;


            // If robot is in the neutral zone OR far alliance zone:
        if ((Robot.alliance == 'B' && projectedOdoX > blueTargetX[2]) || (Robot.alliance == 'R' && projectedOdoX < redTargetX[2])){
            interpolatedTurretOffset = ((Driver_Controller.useOffsets())?Driver_Controller.offsetSlider():0.0);
            desiredSpeed = Math.max(10.0, Math.min(60.0, (Driver_Controller.buttonLimitShuttle()?20.0:50.0)+0*Driver_Controller.shooterOffsetSlider()));
            cowlAngle = 0.0;
            modTurretOff = 180 - (turretErrorDeg + interpolatedTurretOffset + 180 + 360*Math.pow(10, 5))%360;
            inAlliance = true;
            return;
        }
        inAlliance = false;
        //distance += 1.0;

            // *********************
            // interpolate for the speed, cowl angle, and turret offset
            // *********************
        ArrayList<Double[]> upper, lower;
        int currentUpper = -1;
        while ((upper = cowlInterpolation.get(++currentUpper)).get(0)[0] < targetAngleDeg)
            {}
        lower = cowlInterpolation.get(Math.max(0, currentUpper-1));

        Double[] Ldist1, Udist1;
        currentUpper = 1;
        while (upper.get(currentUpper)[0] < distance)
            {currentUpper++;}
        Udist1 = upper.get(currentUpper);
        Ldist1 = upper.get(Math.max(1, currentUpper-1));

        Double[] Ldist2, Udist2;
        currentUpper = 1;
        while (lower.get(currentUpper)[0] < distance)
            {currentUpper++;}
        Udist2 = lower.get(currentUpper);
        Ldist2 = lower.get(Math.max(1, currentUpper-1));

        Double[] interpolated = new Double[3];
 
        for (int i = 1; i <= 3; ++i){

            Double temp1 = Udist1[i] - (Udist1[0]-distance)/(Udist1[0]-Ldist1[0])*(Udist1[i]-Ldist1[i]);
            Double temp2 = Udist2[i] - (Udist2[0]-distance)/(Udist2[0]-Ldist2[0])*(Udist2[i]-Ldist2[i]);
            interpolated[i-1] = temp2 
                //- (upper.get(0)[0]-targetAngleDeg)
                - (targetAngleDeg - lower.get(0)[0])
                /(upper.get(0)[0]-lower.get(0)[0])
                *(temp2-temp1);
            //if (((update_time++) & 7)==0)
            if (false)
                   System.out.printf("udist1 %f %f, udist2 %f %f, distance %f, targetAng %f %f %f, temp1 %f, temp2 %f, interp %f: %f %f %f %f\n",
            Udist1[0], Udist1[i],
            Udist2[0], Udist2[i],
            distance,
            targetAngleDeg,
            upper.get(0)[0], lower.get(0)[0],
            temp1, temp2, interpolated[i-1],
            temp2 ,
                - (upper.get(0)[0]-targetAngleDeg)
                ,(upper.get(0)[0]-lower.get(0)[0])
                ,(temp2-temp1)
            );
        }

            // Incorporate Operator sliders.
        if (!Driver_Controller.manualSwitch())interpolatedTurretOffset = interpolated[2];
        if (Driver_Controller.useOffsets()) interpolatedTurretOffset += Driver_Controller.offsetSlider();
        desiredSpeed = interpolated[1];
        desiredSpeed -= 10;
        if (Driver_Controller.useOffsets()) desiredSpeed += Driver_Controller.shooterOffsetSlider();
        //desiredSpeed = Math.min(60.0, desiredSpeed);
        cowlAngle = interpolated[0];
        if (Driver_Controller.useOffsets()) cowlAngle -= 
            0.2835*Driver_Controller.m_Controller3.getRawAxis(4);
        cowlAngle = Math.max(.24, Math.min(.807, cowlAngle+0.04));
        desiredSpeed = Math.max(10.0, Math.min(60.0, desiredSpeed));

        Double errorFix = 5*(prevAngle-targetAngleDeg);
        if (errorFix > 5*180) errorFix -= 5*360;
        if (errorFix < -5*180) errorFix += 5*360;
        try{Robot.fileWriter.write((Robot.cnt++)+", "+errorFix/5+", "+(180-(turretErrorDeg + interpolatedTurretOffset + 180 + 360*Math.pow(10, 5))%360));Robot.fileWriter.newLine();}catch(Exception e){System.out.println("fail");e.printStackTrace();};
        modTurretOff = 180-(turretErrorDeg + interpolatedTurretOffset - errorFix + 180 + 360*Math.pow(10, 5))%360;//in the range of -180 to 180 degrees
        //System.out.println(modTurretOff);
        }catch (Exception e){}
    }

    public static Double spinTurret(){
        Double maxPowDist = 40.0;
        Double power = modTurretOff/maxPowDist*maxPower;

        if (Math.abs(modTurretOff) < maxError){ // if really close to perfect, then no power needed
            return 0.0;
        }
        
        Double minSpeed = 600.0;//TODO FIXME rpm
        Double maxAdjust = Math.min(1.0, Math.max(-1.0, modTurretOff/maxPowDist)); // the desired max speed changes based on our distance
        Double speedCompensation = 0.1*maxAdjust*(minSpeed-Math.abs(turretEncoder.getVelocity()))/minSpeed;
        power += speedCompensation;
        power = Math.min(maxPower, Math.max(-maxPower, power)); // limit power to between -maxPower and +maxPower	
        return -power/1.5;
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
                    //turretEncoder.setPosition(0.0);
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
            FileReader fileReader = new FileReader(Constants.interpolationReadFile);
            BufferedReader bufferedReader = new BufferedReader(fileReader);
            String line = null;
            int curSubarray = -1;
            Integer numberValues;
            while ((line = bufferedReader.readLine()) != null) {
                switch(line){
                    case "{": // add a new subarray
                        ++curSubarray;
                        ArrayList<Double[]> newAdd = new ArrayList<>(0);
                        cowlInterpolation.add(newAdd);
                        break;
                    case "}":
                        break;
                    default:
                        String[] words = line.trim().split("\\s+");
                        numberValues = words.length;
                        Double[] converted = new Double[numberValues];
                        for (int i = 0; i < numberValues; ++i){
                            converted[i] = Double.parseDouble(words[i]);
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

    static int outerCompare(ArrayList<Double[]> a1, ArrayList<Double[]> a2){
        if (a1.get(0)[0] < a2.get(0)[0]) return -1;
        return 1;
    }
    static int arrayCompare(Double[] a1, Double[] a2) {
        if (a1.length < a2.length) return -1;
        if (a2.length < a1.length) return 1;
        if (a1[0] < a2[0]) return -1;
        return 1;
    }

    static void writeToFile(){
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(Constants.interpolationWriteFile))){
            for (int i = 0; i < cowlInterpolation.size(); ++i){
                System.out.println("{");
                writer.write("{");
                writer.newLine();
                ArrayList<Double[]> curList = cowlInterpolation.get(i);
                for (int j = 0; j < curList.size(); ++j){
                    Double[] curArray = curList.get(j);
                    String toWrite = "";
                    for (int k = 0; k < curArray.length; ++k){
                        toWrite = toWrite + curArray[k] + " ";
                    }
                    System.out.println(toWrite);
                    writer.write(toWrite);
                    writer.newLine();
                }
                System.out.println("}");
                writer.write("}");
                writer.newLine();
            }
        }catch (Exception e) {e.printStackTrace();}
    }
    
    public static void saveInterpolationData(Double[] newData){
        int modNum = Math.toIntExact(Math.round(((newData[0]+32550.0*360+180)%360.0)/45.0))%8;
        Double[] newArray = new Double[newData.length-1];
        for (int i = 1; i < newData.length; ++i)
            newArray[i-1] = newData[i];
        cowlInterpolation.get(modNum).add(newArray);
        for (int i = 0; i < cowlInterpolation.size(); ++i){
            cowlInterpolation.get(i).sort((a1, a2) -> arrayCompare(a1, a2));
        }
        cowlInterpolation.sort((a1, a2) -> outerCompare(a1, a2));
        writeToFile();
    }

    public static Double[] curPower = {0.0, 0.0};
    public static Double[] prevSpeed1 = {0.0, 0.0, 0.0, 0.0, 0.0},
                           prevSpeed2 = {0.0, 0.0, 0.0, 0.0, 0.0};
    public static int speedIndex = 0;

    public static Boolean close = false;
    public static Double avgSpeed = 0.0;

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

        if (velocity < desiredSpeed*0.9 || velocity > desiredSpeed*1.1)
            close = false;

        avgSpeed = velocity/2;
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

        if (velocity < desiredSpeed*0.9 || velocity > desiredSpeed*1.1)
            close = false;
        
        if (close){
            LED.setPattern('B');
        }else{
            LED.setPattern('R');
        }
        avgSpeed += velocity/2;
        speedIndex = nextSpeedIndex;
        return new Double[] {power1, power2};
    }
}
//fix lines that say TODO FIXME for arbitrary constants