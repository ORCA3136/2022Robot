
package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private CANSparkMax innerRoller;
    boolean togglePressed, toggleOn;
    RelativeEncoder midEncoder;

    public Intake(){
        innerRoller = new CANSparkMax(Constants.INTAKE, MotorType.kBrushless);
        midEncoder = innerRoller.getEncoder();
        togglePressed = false;
        toggleOn = false;
     
    }

    @Override   
    public void periodic(){
        //TODO - put data out to the smartdashboard
    }
    public void intakeStop() {
        innerRoller.set(0);
    }

    public void intakeForward(double speed){
        innerRoller.set(speed);
    }
    
    public void intakeReverse(double speed){
        innerRoller.set(-1 * speed);
    }
}
