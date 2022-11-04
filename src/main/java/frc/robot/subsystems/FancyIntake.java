package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.NeutralMode; 
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FancyIntake extends SubsystemBase{
    private VictorSPX leftRoller;
    private TalonSRX rightRoller;
    private CANSparkMax outerRoller;
    private Servo exampleServo = new Servo(1);

    boolean toggleTop;
   // RelativeEncoder midEncoder;
    DigitalInput toplimitSwitch = new DigitalInput(1);
    DigitalInput bottomlimitSwitch = new DigitalInput(2);
   // Joystick joystick = new Joystick(0);
    
    public FancyIntake(){
        leftRoller = new VictorSPX(Constants.INTAKELEFT);
        rightRoller = new TalonSRX(Constants.INTAKERIGHT);
        leftRoller.configFactoryDefault();
        rightRoller.configFactoryDefault();
        leftRoller.setNeutralMode(NeutralMode.Brake);
        rightRoller.setNeutralMode(NeutralMode.Brake);
        outerRoller = new CANSparkMax(Constants.OUTERROLLER, MotorType.kBrushless);

        toggleTop = false;
     //   toggleOn = false;

        leftRoller.setInverted(true);
        rightRoller.setInverted(false);

        leftRoller.follow(rightRoller);

    }       

    public void halt(){
        rightRoller.set(ControlMode.PercentOutput, 0);
       // leftRoller.set(ControlMode.PercentOutput, 0);
    }

    public void servoHand(){
        exampleServo.set(1);
        exampleServo.setAngle(145);

    }

    public void wheelsStop(){
        outerRoller.set(0);
    }
    
    public void wheelsIn(double speed){
        outerRoller.set(speed);
    }

    public void periodic(){
        toggleTop = toplimitSwitch.get();
        SmartDashboard.putBoolean("Top Switch", toggleTop);
        
    }

    public boolean getToggleTop(){
        return toplimitSwitch.get();

    }

    public void deployOuter(double speed) {
        rightRoller.set(ControlMode.PercentOutput, speed);
    }
 
    public void retractOuter(double speed) {
         rightRoller.set(ControlMode.PercentOutput,speed*-1);

    }
}   


