package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FancyIntake extends SubsystemBase{
    private VictorSPX leftRoller;
    private TalonSRX rightRoller;
    private CANSparkMax outerRoller;

    boolean togglePressed, toggleOn;
    RelativeEncoder midEncoder;
    DigitalInput toplimitSwitch = new DigitalInput(0);
    DigitalInput bottomlimitSwitch = new DigitalInput(1);
    Joystick joystick = new Joystick(0);
    
    public FancyIntake(){
        leftRoller = new VictorSPX(Constants.INTAKELEFT);
        rightRoller = new TalonSRX(Constants.INTAKERIGHT);
        outerRoller = new CANSparkMax(Constants.OUTERROLLER, MotorType.kBrushless);
        togglePressed = false;
        toggleOn = false;

        leftRoller.setInverted(true);
        rightRoller.setInverted(false);

        leftRoller.follow(rightRoller);

    }       

    public void halt(){
        rightRoller.set(ControlMode.PercentOutput, 0);
    }

    public void wheelsStop(){
        outerRoller.set(0);
    }
    
    public void wheelsIn(double speed){
        outerRoller.set(-1 * speed);
    }

    public void deployOuter(double speed) {
        if (speed > 0) {} 
        else {
            if (bottomlimitSwitch.get()) {
                // We are going down and bottom limit is tripped so stop
                rightRoller.set(ControlMode.PercentOutput,0);
            } else {
                // We are going down but bottom limit is not tripped so go at commanded speed
                rightRoller.set(ControlMode.PercentOutput,speed);
            }
        }
    }
    
    public void retractOuter(double speed) {
        if (speed > 0) {
            if (toplimitSwitch.get()) {
                // We are going up and top limit is tripped so stop
                rightRoller.set(ControlMode.PercentOutput,0);

            } else {
                // We are going up but top limit is not tripped so go at commanded speed
                rightRoller.set(ControlMode.PercentOutput,speed*-1);
            }
    }

    }
    }   


