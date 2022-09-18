package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class OperatorInterface extends SubsystemBase {
    private final Joystick leftStick;
    private final Joystick rightStick;
    
    
    public OperatorInterface(){
        this.leftStick = new Joystick(Constants.LeftJoystickPort);
        this.rightStick = new Joystick(Constants.RightJoystickPort);


    }

    public double getLeftY() {
        double ret = leftStick.getY();
        if (Math.abs(ret) > Constants.JoystickDeadband) {
          return ret;
        }
    
        return 0.0;
      }
    
      public double getRightY() {
        double ret = rightStick.getY();
        if (Math.abs(ret) > Constants.JoystickDeadband) 
        {
          return ret;
        }
    
        return 0.0;
      }

      public boolean getLeftButton(int button) {
        return this.leftStick.getRawButton(button);
      }
    
      public boolean getRightButton(int button) {
        return this.rightStick.getRawButton(button);
      }
    
}
