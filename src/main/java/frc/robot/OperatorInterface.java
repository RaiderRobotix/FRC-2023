package frc.robot;

import edu.wpi.first.wpilibj.Joystick;


public class OperatorInterface {
    private final Joystick leftStick;
    private final Joystick rightStick;
    
    
    private OperatorInterface(){
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
