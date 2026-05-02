package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;

public class ControlState {
    public Boolean isRed;
    public Double xPos;
    public Double yPos;
    public Double targetAngle;
    public Double xJoy;
    public Double yJoy;
    public Boolean buttonLimitShuttle;
    public Boolean buttonRestrictTransport;
    public Boolean manualSwitch;
    public Boolean runShooterSwitch;
    public Boolean transportSwitch;
    public Boolean intakeSwitch;
    public Boolean pauseTurret;
    public Boolean buttonShooterReverse;
    public Boolean buttonIntakeReverse;
    public Boolean buttonTransportReverse;
    public Boolean buttonResetTurret;
    public Boolean buttonEBrake;
    public int kitchenStove;

    public ControlState(){
        isRed = (DriverStation.getAlliance().toString().charAt(9) == 'R');
        buttonEBrake = Driver_Controller.buttonEBrake();
        buttonIntakeReverse = Driver_Controller.buttonIntakeReverse();
        buttonLimitShuttle = Driver_Controller.buttonLimitShuttle();
        buttonResetTurret = Driver_Controller.buttonResetTurret();
        buttonRestrictTransport = Driver_Controller.buttonRestrictTransport();
        buttonShooterReverse = Driver_Controller.buttonShooterReverse();
        buttonTransportReverse = Driver_Controller.buttonTransportReverse();
        intakeSwitch = Driver_Controller.intakeSwitch();
        kitchenStove = Driver_Controller.kitchenStove();
        manualSwitch = Driver_Controller.manualSwitch();
        pauseTurret = Driver_Controller.pauseTurret();
        runShooterSwitch = Driver_Controller.runShooterSwitch();
        targetAngle = Rotary_Controller.RotaryJoystick(Driver_Controller.m_Controller1)+RobotContainer.rotaryOffset;
        transportSwitch = Driver_Controller.transportSwitch();
        xJoy = Driver_Controller.SwerveXPassthrough;
        xPos = RobotContainer.drivetrain.getState().Pose.getX();
        yJoy = Driver_Controller.SwerveYPassthrough;
        yPos = RobotContainer.drivetrain.getState().Pose.getY();
    }

    public void update(String line){
        String[] words = line.trim().split(" ");
        if (words.length > 50) return;
        buttonEBrake = Boolean.parseBoolean(words[0]);
        buttonIntakeReverse = Boolean.parseBoolean(words[1]);
        buttonLimitShuttle = Boolean.parseBoolean(words[2]);
        buttonResetTurret = Boolean.parseBoolean(words[3]);
        buttonRestrictTransport = Boolean.parseBoolean(words[4]);
        buttonShooterReverse = Boolean.parseBoolean(words[5]);
        buttonTransportReverse = Boolean.parseBoolean(words[6]);
        intakeSwitch = Boolean.parseBoolean(words[7]);
        kitchenStove = Integer.parseInt(words[8]);
        manualSwitch = Boolean.parseBoolean(words[9]);
        pauseTurret = Boolean.parseBoolean(words[10]);
        runShooterSwitch = Boolean.parseBoolean(words[11]);
        targetAngle = Double.parseDouble(words[12]);
        transportSwitch = Boolean.parseBoolean(words[13]);
        xJoy = Double.parseDouble(words[14]);
        xPos = Double.parseDouble(words[15]);
        yJoy = Double.parseDouble(words[16]);
        yPos = Double.parseDouble(words[17]);
        isRed = Boolean.parseBoolean(words[18]);
    }

    public String getString(){
        String ret = buttonEBrake + " " + 
        buttonIntakeReverse + " " + 
        buttonLimitShuttle + " " + 
        buttonResetTurret + " " + 
        buttonRestrictTransport + " " + 
        buttonShooterReverse + " " + 
        buttonTransportReverse + " " + 
        intakeSwitch + " " + 
        kitchenStove + " " + 
        manualSwitch + " " + 
        pauseTurret + " " + 
        runShooterSwitch + " " + 
        targetAngle + " " + 
        transportSwitch + " " + 
        xJoy + " " + 
        xPos + " " + 
        yJoy + " " + 
        yPos + " " +
        isRed;
        return ret;
    }
}