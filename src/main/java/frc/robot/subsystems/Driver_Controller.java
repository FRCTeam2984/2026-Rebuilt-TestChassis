/*
 * Subsystem to run at RobotInit to read and auto set the controllers
 * so they are configured correctly before driving the robot and 
 * without user intervention.
 * 
 * Also used to return button values to other subsystems instead of 
 * previous RobotContainer method.
 */
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;


public class Driver_Controller {
    
    /*
     * Buttons 11 and 12 off   = Drive Joystick
     * Button 11 off and 12 on = Rotary Encoder and Buttons
     * Button 11 on and 12 off = Rotary Switch and Buttons
     * Buttons 11 and 12 on    = Right Side Buttons
     * 
     * m_Controller0           = Drive Joystick
     * m_Controller1           = Rotary Encoder
     * m_Controller2           = Rotary Switch and Buttons
     * m_Controller3           = 
     */
    public static CommandXboxController m_Controller0 = new CommandXboxController(0); //Set Temp value to complete initialization of swervedrive
    public static Joystick m_Controller1 = new Joystick(1);
    public static XboxController m_Controller2 = new XboxController(0);
    public static XboxController m_Controller3 = new XboxController(2);
    private static XboxController m_tempController;
    public static int SwerveCommandXboxControllerPort; // Value of joystick for controlling swerve drive
    public static int SwerveRotaryEncoderPort;         // Value of rotary encoder for controlling swerve drive

    public static boolean SwerveCommandControl = false;// Controls Swerve command shifting, false = Controller and true = Command
    public static double SwerveEncoderPassthrough;     // Encoder pass back to swervedrive system
    public static double SwerveXPassthrough;           // X value pass back to swervedrive system
    public static double SwerveYPassthrough;           // Y value pass back to swervedrive system
    public static double SwerveCommandEncoderValue;    // Command requested encoder value for swervedrive
    public static double SwerveCommandXValue;          // Command requested X value for swervedrive
    public static double SwerveCommandYValue;          // Command requested Y value for swervedrive

    public static void define_Controller(){
    for(int i = 0; i < 3; ++i){
        m_tempController = new XboxController(i);
        if (m_tempController.getRawButton(14) == true){
            m_Controller0 = new CommandXboxController(i);
            m_Controller1 = new Joystick(i);
        }
        else if (m_tempController.getRawButton(13) == true){
            m_Controller3 = m_tempController;
        }
        else{
            m_Controller2 = m_tempController;
        }
    }
}

public static Boolean driverSwitch(){
    return m_Controller1.getRawButton(6);}
public static Boolean buttonReefAlign(){
    return m_Controller1.getRawButton(1);}
public static Boolean flipDrive(){
    return m_Controller1.getRawButton(3);}
public static Boolean buttonResetIntake(){
    return false;}
public static Boolean buttonHPSalign(){
    return m_Controller1.getRawButton(2);}
public static Boolean buttonRemoveAlign(){
    return m_Controller1.getRawButton(5);}
public static Double upperDriverSlider(){
    return m_Controller1.getRawAxis(3);}
public static Double lowerDriverSlider(){
    return m_Controller1.getRawAxis(4);}
    
public static Boolean switch1(){
    return !m_Controller3.getRawButton(8);}
public static Boolean switch2(){
    return !m_Controller3.getRawButton(7);}
public static Boolean switch3(){
    return !m_Controller3.getRawButton(9);}
public static Boolean switch4(){
    return !m_Controller3.getRawButton(10);}
public static Boolean manualSwitch(){
    return switch1();}
public static Boolean runShooterSwitch(){
    return switch2();}
public static Boolean transportSwitch(){
    return switch3();}
public static Boolean intakeSwitch(){
    return switch4();}

public static Double cowlSlider(){
    return 0.38*(1+m_Controller3.getRawAxis(4));}
public static Double shooterSpeedSlider(){
    return 35+25*m_Controller3.getRawAxis(3);}
public static Double offsetSlider(){
    return 20*m_Controller3.getRawAxis(1);}

public static Boolean useOffsets(){
    return m_Controller3.getRawButton(1);}
public static Boolean pauseTurret(){
    return  m_Controller3.getRawButton(2);}
public static Boolean buttonShooterReverse(){
    return m_Controller3.getRawButton(3);}
public static Boolean buttonIntakeReverse(){
    return m_Controller3.getRawButton(4);}
public static Boolean buttonTransportReverse(){
    return m_Controller3.getRawButton(5);}
public static Boolean buttonResetTurret(){
    return m_Controller3.getRawButton(6);}

public static Boolean buttonEBrake(){
    return (m_Controller2.getRawButton(1) || m_Controller1.getRawButton(6));}
public static Boolean buttonBrake(){
    return (m_Controller1.getRawButton(1));}
public static Trigger needBrake = new Trigger(() -> (buttonEBrake() || buttonBrake()));

public static Double shooterOffsetSlider(){
    return 10*m_Controller3.getRawAxis(3);}


final  double pos[] = {-1.0,-0.75,-0.5,-0.1 ,-0.03, 0,0.03, 0.1, 0.5, 0.75,1};
final  double pwr[] = {-1  , -0.3,-0.1,-0.02,    0, 0,   0,0.02, 0.1,   .3,1};
final double Speed = new RobotContainer().MaxSpeed;
public  double joystick_curve(double joy) {
    int i;
    if (joy<=-1) joy=-1;
    if (joy>=1) joy=1;
    for (i=0;i<10;i++) {
    if ((pos[i]<=joy) && (pos[i+1]>=joy)) {
        //System.out.println( ((joy-pos[i]) / (pos[i+1]-pos[i]) * (pwr[i+1]-pwr[i]) + pwr[i]) * MaxSpeed);
        return(((joy-pos[i]) / (pos[i+1]-pos[i]) * (pwr[i+1]-pwr[i]) + pwr[i]) * Speed/*RobotContainer.MaxSpeed*/);
        }
    }
    return(0);
}
public double[] betterJoystickCurve(double x, double y) {
    double radius = Math.sqrt((x*x)+(y*y));
    double angle = Math.atan2(y, x);
    radius = joystick_curve(radius);
    double[] returnValue = {Math.sin(angle)*radius, Math.cos(angle)*radius};
    return returnValue;
}

public static Double pigeonOffset = 0.0;

public static void SwerveControlSet(boolean command){
    if ((SwerveCommandControl == false) && command == true){
        pigeonOffset = ((RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees()+ 360*1000 + 180)%360)-180-RobotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble();
    }
    SwerveCommandControl = command;
    SwerveInputPeriodic();
}
public static void SwerveInputPeriodic(){
    if (SwerveCommandControl){ // Command Mode
        SwerveEncoderPassthrough = SwerveCommandEncoderValue-pigeonOffset;
        SwerveXPassthrough = SwerveCommandXValue;
        SwerveYPassthrough = SwerveCommandYValue;
    }
    else{ //Controller Mode
        Double sliderMult = Driver_Controller.upperDriverSlider()*0.45+0.55;
        //SwerveEncoderPassthrough = Rotary_Controller.RotaryJoystick(m_Controller1);
        SwerveXPassthrough = -RobotContainer.betterJoystickCurve(m_Controller0.getLeftX()+0.04, m_Controller0.getLeftY()-0.07)[0]*sliderMult*((flipDrive())?-1.0:1.0);
        SwerveYPassthrough = -RobotContainer.betterJoystickCurve(m_Controller0.getLeftX()+0.04, m_Controller0.getLeftY()-0.07)[1]*sliderMult*((flipDrive())?-1.0:1.0);
        SwerveEncoderPassthrough = Math.toDegrees(Math.atan2(SwerveXPassthrough, SwerveYPassthrough));
        //System.out.println(Math.toDegrees(Math.atan2(SwerveXPassthrough, SwerveYPassthrough)));
        //SwerveXPassthrough = -m_Controller0.getLeftY();
        //SwerveYPassthrough = -m_Controller0.getLeftX();
    }
}


}
