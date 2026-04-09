package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;

import frc.robot.Constants;

public class Transport {
    public static TalonFX transportMotor;
    public static TalonSRX intakeMotor;
    public static TalonSRX agitatorMotor;
    public static TalonFX spindexerMotor;
    public static Boolean intakeMotorWorking = false, transportMotorWorking = false, agitatorMotorWorking = false, spindexerMotorWorking = false;
    public static Servo agitator1 = new Servo(Constants.agitator1ID), agitator2 = new Servo(Constants.agitator2ID);

    public static void initMotors(){
        try{
            transportMotor = new TalonFX(Constants.transportID);
            transportMotorWorking = true;
        }catch(Exception e){}
        try{
            intakeMotor = new TalonSRX(Constants.intakeID);
            intakeMotorWorking = true;
        }catch(Exception e){}
        try{
            agitatorMotor = new TalonSRX(Constants.agitatorRedlineID);
            agitatorMotorWorking = true;
        }catch(Exception e){}
        try{
            spindexerMotor = new TalonFX(Constants.spindexerFalconID);
            spindexerMotorWorking = true;
        }catch(Exception e){}
    }
    //public static RelativeEncoder encoder = neoMotor.getEncoder(); // creating an encoder for the motor called motor
    public static Double spinIntake(){
        if (Driver_Controller.buttonIntakeReverse()){
            return -0.4;
        }else if(Driver_Controller.intakeSwitch()){
            return 0.7;//TODO FIXME
        }else{
            return 0.0;
        }
    }
    public static Double[] powerArray = {0.45/6.0, 0.6/6.0, 0.7/6.0, 0.85/6.0, 1.0/6, 1.5/6, 2.0/6, 3.5/6};

    public static Double spinTransport(){
        agitate(Driver_Controller.transportSwitch() || Driver_Controller.buttonTransportReverse());
        if (Driver_Controller.buttonTransportReverse())
            return 0.25;
        if (!Driver_Controller.runShooterSwitch())
            return 0.0;
        if (Driver_Controller.transportSwitch()){
            return -0.85;//-powerArray[Driver_Controller.kitchenStove()]/1; // 1.5 and 1.3 still has balls get stuck, 5:1 gearbox
            //return 0.35; // 4:1 gearbox
        }
        return 0.0;
    }
    public static Double spindexerPower(){
        if (Driver_Controller.buttonTransportReverse() || Driver_Controller.buttonShooterReverse())
            return -0.2;
        if (!Driver_Controller.runShooterSwitch())
            return 0.0;
        if (Driver_Controller.transportSwitch()){
            return powerArray[Driver_Controller.kitchenStove()];//4.25;
        }
        return 0.0;
    }

    // the following two functions are to make sure that the same power setting is repeatedly sent to the motors
    // they also are part of the solution to not needing to comment out this entire file
    public static Double prevIntakePower = 0.0;
    public static void setIntake(Double power){
        if (intakeMotorWorking == false){
            return;
        }
        // check if the next power is within 5 percent of the current one
        if (((prevIntakePower-power)*20 > prevIntakePower)
        || ((power-prevIntakePower)*20 > prevIntakePower)){
            try{
                intakeMotor.set(TalonSRXControlMode.PercentOutput, power);
                prevIntakePower = power;
            }catch(Exception e){}
        }
    }

    public static Double prevSpindexerPower = 0.0;
    public static void setSpindexer(Double power){
        power = -power;
        if (spindexerMotorWorking == false){
            return;
        }
        // check if the next power is within 5 percent of the current one
        if (((prevSpindexerPower-power)*20 > prevSpindexerPower)
        || ((power-prevSpindexerPower)*20 > prevSpindexerPower)){
            try{
                spindexerMotor.set(power);
                prevSpindexerPower = power;
            }catch(Exception e){}
        }
    }

    public static Double prevTransportPower = 0.0;
    public static void setTransport(Double power){
        if (transportMotorWorking == false){
            return;
        }
        // check if the next power is within 5 percent of the current one
        if (((prevTransportPower-power)*20 > prevIntakePower)
        || ((power-prevTransportPower)*20 > prevIntakePower)){
            try{
                transportMotor.set(power);
                prevTransportPower = power;
            }catch(Exception e){}
        }
    }

    public static void agitate(Boolean isActive){
        if (isActive || (prevIntakePower != 0.0)) {
            if ((Driver_Controller.transportSwitch() || prevIntakePower > 0) && !Driver_Controller.buttonTransportReverse())
                agitatorMotor.set(TalonSRXControlMode.PercentOutput, 0.8);
            else
                agitatorMotor.set(TalonSRXControlMode.PercentOutput, -0.8);
        } else {
            agitatorMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
        }
    }
}
