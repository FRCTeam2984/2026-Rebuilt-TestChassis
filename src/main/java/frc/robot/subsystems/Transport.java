package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class Transport {
    public static TalonSRX transportMotor;
    public static TalonSRX intakeMotor;
    public static Boolean intakeMotorWorking = false, transportMotorWorking = false;

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
    }
    //public static RelativeEncoder encoder = neoMotor.getEncoder(); // creating an encoder for the motor called motor
    public static Double spinIntake(){
        if(Driver_Controller.buttonL3()){//TODO FIXME
            return 0.7;//TODO FIXME
        }else{
            return 0.0;
        }
    }
    public static Double spinTransport(){
        if (Driver_Controller.buttonL4()){//TODO FIXME
            return -0.5; // TODO FIXME
        }else if (Driver_Controller.buttonTransportPivot()){
            return 0.5; // TODO FIXME
        }else{
            return 0.0;
        }
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
}
