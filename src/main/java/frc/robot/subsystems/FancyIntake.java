package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FancyIntake extends SubsystemBase{
    private TalonSRX leftRoller;
    private TalonSRX rightRoller;

    boolean togglePressed, toggleOn;
    RelativeEncoder midEncoder;
    DigitalInput toplimitSwitch = new DigitalInput(0);
    DigitalInput bottomlimitSwitch = new DigitalInput(1);
    Joystick joystick = new Joystick(0);
    
    public FancyIntake(){
        leftRoller = new TalonSRX(Constants.INTAKELEFT);
        rightRoller = new TalonSRX(Constants.INTAKERIGHT);
        togglePressed = false;
        toggleOn = false;
    }       

    public void teleopPeriodic() {
    setMotorSpeed(joystick.getRawAxis(2));
}

    public void setMotorSpeed(double speed) {
        if (speed > 0) {
            if (toplimitSwitch.get()) {
                // We are going up and top limit is tripped so stop
                leftRoller.set(ControlMode.PercentOutput, 0);
                rightRoller.set(ControlMode.PercentOutput,0);

            } else {
                // We are going up but top limit is not tripped so go at commanded speed
                leftRoller.set(ControlMode.PercentOutput,speed);
                rightRoller.set(ControlMode.PercentOutput,speed);
            }
        } else {
            if (bottomlimitSwitch.get()) {
                // We are going down and bottom limit is tripped so stop
                leftRoller.set(ControlMode.PercentOutput,0);
                rightRoller.set(ControlMode.PercentOutput,0);
            } else {
                // We are going down but bottom limit is not tripped so go at commanded speed
                leftRoller.set(ControlMode.PercentOutput,speed);
                rightRoller.set(ControlMode.PercentOutput,speed);
            }
        }
    }
    

    }


