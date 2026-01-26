package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class Turret {
    public static Double distance;
    public static Double[] blueTargetX = {1.988947, 1.988947, 4.62534}, blueTargetY = {2.0173315, 6.05119945, 4.034663}, redTargetX = {14.552041, 14.552041, 11.915394}, redTargetY = {2.0173315, 6.05119945, 4.034663};
    public static Servo servo1 = new Servo(Constants.servo1ID);
    //servo1.setAngle(50.0);
    //public static SparkMax turretSpin = new SparkMax(Constants.turretSpinID, MotorType.kBrushless);
    public static void calcDist(){/*
        Double odoY = RobotContainer.drivetrain.getState().Pose.getY();
        Double odoX = RobotContainer.drivetrain.getState().Pose.getX();
        Double odoA = Math.toRadians(RobotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble());
        if ()
        destX = destination x
        destY = destination y
        turretOffX = turret offset of x
        turretOffY = turret offset of y
        turretAngle = turret angle
        shooting = false

        Double turrX, turrY;

        turrX = odoX + turretOffX*Math.cos(odoA in radians) + turretOffY*Math.sin(odoA in radians) - destX
        turrY = odoY + turretOffX*Math.sin(odoA in radians) + turretOffY*Math.cos(odoA in radians) - destY

        //might need to change cos and sin above

        distance = Math.sqrt(Math.pow(turrX , 2) + Math.pow(turrY, 2));
        // targetAngle (might be off by a negative or multiple of 90)
        targetAngle = Math.atan(turrX / turrY)-odoA;
        turretOffset = targetAngle - turretAngle;
        modTurretAngle = 180 - (turretOffset + 180 + 360*106)%360;*/
    }
}
