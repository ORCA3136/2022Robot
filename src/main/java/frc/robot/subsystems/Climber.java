package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


public class Climber extends SubsystemBase{
    private VictorSPX climber;
    RelativeEncoder climbEncoder;



   public Climber(){
       climber = new VictorSPX(15);
       

   }

   public void raiseClimber(double climberSpeed){
       climber.set(ControlMode.PercentOutput, climberSpeed);
   }

   public void lowerClimber(double climberSpeed){
       climber.set(ControlMode.PercentOutput,-1 * climberSpeed);
   }

   public void stopClimber(){
       climber.set(ControlMode.PercentOutput, 0.0);
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
