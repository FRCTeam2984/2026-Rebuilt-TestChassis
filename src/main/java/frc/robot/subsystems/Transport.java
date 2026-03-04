package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;

import frc.robot.Constants;

public class Transport {
    public static TalonSRX transportMotor;
    public static TalonSRX intakeMotor;
    public static TalonSRX agitatorMotor;
    public static Boolean intakeMotorWorking = false, transportMotorWorking = false, agitatorMotorWorking = false;
    public static Servo agitator1 = new Servo(Constants.agitator1ID), agitator2 = new Servo(Constants.agitator2ID);

    public static void initMotors(){
        try{
            transportMotor = new TalonSRX(Constants.transportID);
            transportMotorWorking = true;
        }catch(Exception e){
            System.out.println("no transport motor");
        }
        try{
            intakeMotor = new TalonSRX(Constants.intakeID);
            intakeMotorWorking = true;
        }catch(Exception e){
            System.out.println("no intake motor");
        }
        try{
            agitatorMotor = new TalonSRX(Constants.agitatorRedlineID);
            agitatorMotorWorking = true;
        }catch(Exception e){
            System.out.println("no agitator motor");
        }
    }
    //public static RelativeEncoder encoder = neoMotor.getEncoder(); // creating an encoder for the motor called motor
    public static Double spinIntake(){
        if (Driver_Controller.buttonIntakeReverse()){
            return -0.4;
        }else if(Driver_Controller.intakeSwitch()){
            return 0.4;//TODO FIXME
        }else{
            return 0.0;
        }
    }
    public static Double spinTransport(){
        agitate(Driver_Controller.transportSwitch());
        if (Driver_Controller.transportSwitch()){
            if (Driver_Controller.buttonTransportReverse()){
                return 0.5;
            }
            return -0.65;
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

    public static Double prevTransportPower = 0.0;
    public static void setTransport(Double power){
        power = -power;
        if (transportMotorWorking == false){
            return;
        }
        // check if the next power is within 5 percent of the current one
        if (((prevTransportPower-power)*20 > prevIntakePower)
        || ((power-prevTransportPower)*20 > prevIntakePower)){
            try{
                transportMotor.set(TalonSRXControlMode.PercentOutput, power);
                prevTransportPower = power;
            }catch(Exception e){}
        }
    }

    public static void agitate(Boolean isActive){
        long time = System.nanoTime()/600/1000/1000;
        long time_qsec = System.nanoTime()/250/1000/1000;
        if ((prevIntakePower!=0.0) || (isActive)) {
            if ((time_qsec&15)<=1 || prevTransportPower > 0 || prevIntakePower > 0)
                agitatorMotor.set(TalonSRXControlMode.PercentOutput, -0.6);
            else
                agitatorMotor.set(TalonSRXControlMode.PercentOutput, 0.6);
        } else {
            agitatorMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
        }
        /*
        if (!isActive){
            agitator1.set(0.5);
            agitator2.set(0.5);
            return;
        }
        agitator2.set(0.0);
        if (time%2 == 0){
            // agitator1.set(0.0);
        }else{
            // agitator1.set(1.0);
        }*/
    }
}
