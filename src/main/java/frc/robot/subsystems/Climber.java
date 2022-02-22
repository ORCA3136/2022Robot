package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;


public class Climber extends SubsystemBase{
    private CANSparkMax climber;
    RelativeEncoder climbEncoder;



   public Climber(){
       climber = new CANSparkMax(Constants.Climber1, MotorType.kBrushless);
       
       climbEncoder = climber.getEncoder();

   }

   public void raiseClimber(double climberSpeed){
       climber.set(climberSpeed);
   }

   public void lowerClimber(double climberSpeed){
       climber.set(-1 * climberSpeed);
   }

   public void stopClimber(double climberSpeed){
       climber.set(0);
   }

   public void NukeClimb(XboxController Driver){
   if(Driver.getXButton())
   {
       raiseClimber(1);
   }
   
   else
   {
        lowerClimber(-11);
   }
   }
   
}
