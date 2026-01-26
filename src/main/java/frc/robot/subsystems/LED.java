package frc.robot.subsystems;
import edu.wpi.first.wpilibj.SerialPort;

public class LED {
    public static SerialPort arduino = new SerialPort(9600, SerialPort.Port.kUSB);

    public static char curPattern = 'w';

    public static void setPattern(char newPattern){
        if (newPattern >= 'A' && newPattern <= 'Z'){ // if is a capital letter
            newPattern -= 'a'-'A'; // change to lowercase
        }
        if (newPattern >= 'a' && newPattern <= 'z'){ // if is a latter
            if (newPattern == curPattern){ // don't send the same pattern multiple times
                return;
            }
            curPattern = newPattern;
            System.out.println(newPattern);
            arduino.writeString(""+newPattern);
        }
    }

    public static void setPattern(String newPattern){
        switch(newPattern){
            case "rainbow":
            case "Rainbow":
                setPattern('w'); // rainbow pattern
                break;
            case "blue":
            case "Blue":
                setPattern('b'); // solid blue
                break;
            case "red":
            case "Red":
                setPattern('r'); // solid red
                break;
        }
    }
}
