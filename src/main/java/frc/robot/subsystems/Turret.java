package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class Turret {
    public static Double distance = 0.0, modTurretOff = 0.0; // current distance from the target point, error in target angle
    public static Double[] blueTargetX = {1.988947, 1.988947, 4.62534}, blueTargetY = {2.0173315, 6.05119945, 4.034663}, redTargetX = {14.552041, 14.552041, 11.915394}, redTargetY = {2.0173315, 6.05119945, 4.034663};
    public static Servo servo1 = new Servo(Constants.servo1ID);
    public static Double turretOffX = 0.1, // offset of the turret in meters from the center of the robot, forward is positive
                         turretOffY = 0.0; // left is positive
    //servo1.setAngle(50.0);
    public static SparkMax turretSpin = new SparkMax(Constants.turretSpinID, MotorType.kBrushless);
    public static RelativeEncoder turretEncoder = turretSpin.getEncoder();

    public static void calcDist(){
        // grab odometry values
        Double odoY = RobotContainer.drivetrain.getState().Pose.getY();
        Double odoX = RobotContainer.drivetrain.getState().Pose.getX();
        Double odoA = Math.toRadians(RobotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble());

        Double destX, destY; // find where our destination is
        if (Robot.alliance == 'B'){// if alliance is blue
            if (odoX < blueTargetX[2]){ // if we are in blue alliance zone
                destX = blueTargetX[2];
                destY = blueTargetY[2];
            }else{ // in neutral zone/red alliance zone
                destX = blueTargetX[0];
                destY = blueTargetY[((odoY > blueTargetY[2])?1:0)]; // make it the lower/bigger y value based on if cur y value is above or below hub value
            }
        }else{ // if alliance is red
            if (odoX > redTargetX[2]){ // if we are in red alliance zone
                destX = redTargetX[2];
                destY = redTargetY[2];
            }else{ // in neutral zone/blue alliance zone
                destX = redTargetX[0];
                destY = redTargetY[((odoY > redTargetY[2])?1:0)]; // make it the lower/bigger y value based on if cur y value is above or below hub value
            }
        }
        
        Double turretAngle = turretEncoder.getPosition()*1605;
        Double turrX, turrY; // variables that represent position of the TURRET relative to the target location

        turrX = odoX + turretOffX*Math.cos(odoA) - turretOffY*Math.sin(odoA) - destX;
        turrY = odoY + turretOffX*Math.sin(odoA) + turretOffY*Math.cos(odoA) - destY;
        // calculate above by odometry value and then trigonometric ratios using the robot angle to find turret pos on the field, then subtract destination values

        distance = Math.sqrt(Math.pow(turrX , 2) + Math.pow(turrY, 2)); // pythagorean theorum
        Double targetAngle = Math.atan2(turrX, turrY)-Math.toDegrees(odoA); 
        Double turretOffset = targetAngle - turretAngle;
        modTurretOff = 180 - (turretOffset + 180 + 360*106)%360;
    }
}
