
package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class InnerIntake extends SubsystemBase{
    private CANSparkMax innerRoller;
    boolean togglePressed, toggleOn;
    RelativeEncoder midEncoder;

    public InnerIntake(){
        innerRoller = new CANSparkMax(Constants.kIntake1, MotorType.kBrushless);
        midEncoder = innerRoller.getEncoder();
        togglePressed = false;
        toggleOn = false;
    }

    @Override   
    public void periodic(){
        //TODO - put data out to the smartdashboard
    }
    
    public void intakeIn() {
        innerRoller.set(Constants.kConveyorSpeed);
    }

    public void intakeOut() {
        innerRoller.set(Constants.kConveyorSpeed * -1);
    }

    public void intakeStop() {
        innerRoller.set(0);
    }


    

}
