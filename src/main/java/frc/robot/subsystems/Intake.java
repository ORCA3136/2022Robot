package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase {

    DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kIntakeForward, Constants.kIntakeReverse);

    private CANSparkMax bottomRoller;
    private CANSparkMax midRoller;
    private static CANSparkMax topRoller;
    Conveyor m_convey;
    boolean togglePressed, toggleOn;
    RelativeEncoder topEncoder, bottomEncoder, midEncoder;  

    public Intake(Conveyor conv){
        bottomRoller = new CANSparkMax(Constants.kIntake1, MotorType.kBrushless);
        topRoller = new CANSparkMax(Constants.kIntake2, MotorType.kBrushless);
        midRoller = new CANSparkMax(Constants.kIntake3, MotorType.kBrushless);
        topEncoder = topRoller.getEncoder();
        bottomEncoder = bottomRoller.getEncoder();
        midEncoder = bottomRoller.getEncoder();
        m_convey = conv;
        togglePressed = false;
        toggleOn = false;
    }

    @Override   
    public void periodic(){

        SmartDashboard.putNumber( "Bottom Roller Velocity" , bottomEncoder.getVelocity());
        SmartDashboard.putNumber( "Top Roller Velocity" , topEncoder.getVelocity());
        SmartDashboard.putNumber( "Bottom Roller Current" , bottomRoller.getOutputCurrent());
        SmartDashboard.putNumber( "Top Roller Current" , topRoller.getOutputCurrent());
    }
    
    public void intakeIn() {
        topRoller.set(Constants.kConveyorSpeed);
        bottomRoller.set(Constants.kConveyorSpeed * -1);
        midRoller.set(Constants.kConveyorSpeed * -1);
    }

    public void intakeOut() {
        bottomRoller.set(Constants.kConveyorSpeed);
        topRoller.set(Constants.kConveyorSpeed * -1);
        midRoller.set(Constants.kConveyorSpeed);
    }

    public void intakeStop() {
        bottomRoller.set(
            0);
        topRoller.set(
            0);
            m_convey.stopConveyor();
    }

    public void deployIntake(){
        intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    
    public void retractIntake(){
        intakeSolenoid.set(Value.kReverse);
    }

    public void off(){
        intakeSolenoid.set(Value.kOff);
    }

    public void Toggle(){
        updateToggle();
        if(toggleOn){
            deployIntake();
        }
        else{
            retractIntake();
        }
    }

    public void updateToggle(){
        if(!togglePressed){
            toggleOn = !toggleOn;
            togglePressed = true;
        }
        else{
            togglePressed = true;
        }
    }

}
