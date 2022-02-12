package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase {

   // DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kIntakeForward, Constants.kIntakeReverse);

    private CANSparkMax intakeRoller; 
    boolean togglePressed, toggleOn;
    RelativeEncoder intakeEncoder;  
    private DoubleSolenoid intakeSolenoid;


    public Intake(){
        intakeRoller = new CANSparkMax(Constants.kOutterIntake, MotorType.kBrushless);
       
        intakeEncoder = intakeRoller.getEncoder();
   
        togglePressed = false;
        toggleOn = false;
    }

    @Override   
    public void periodic(){

        SmartDashboard.putNumber( "Intake Roller Current" , intakeRoller.getOutputCurrent());
    }
    
    public void intakeIn(double ConveyorSpeed) {
        intakeRoller.set(ConveyorSpeed);
    }

    public void intakeOut(double ConveyorSpeed) {
        intakeRoller.set(ConveyorSpeed);
    }

    public void intakeStop() {
        intakeRoller.set(0);
  
    }

    public void intakeForward(double speed){
        intakeRoller.set(speed);
    }
    
    public void intakeReverse(double speed){
        intakeRoller.set(speed);
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
