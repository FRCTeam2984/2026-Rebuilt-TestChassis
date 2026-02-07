package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class Transport {
    public static TalonSRX transportMotor = new TalonSRX(Constants.transportID);
    public static TalonSRX intakeMotor = new TalonSRX(Constants.intakeID);
    //public static RelativeEncoder encoder = neoMotor.getEncoder(); // creating an encoder for the motor called motor
    public static void spinIntake(){
        if(Driver_Controller.buttonL3()){//TODO FIXME
            intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.1);//TODO FIXME
        }
        else{
            intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
        }
    }
    public static void spinTransport(){
        if(Driver_Controller.buttonL4()){//TODO FIXME
            transportMotor.set(TalonSRXControlMode.PercentOutput, 0.1);//TODO FIXME
            //do smth about reverse eventually
        }
        else{
            transportMotor.set(TalonSRXControlMode.PercentOutput, 0);
        }
    }
}
