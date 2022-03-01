package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
 

public class FlyWheel extends SubsystemBase {



    public CANSparkMax flyWheel1;
    private  CANSparkMax flyWheel2;
    private CANSparkMax flyWheel3;
    RelativeEncoder flyWheelEncoder;


    public FlyWheel(){
        flyWheel1 = new CANSparkMax(Constants.FlyWheel1, MotorType.kBrushless);
        flyWheel2 = new CANSparkMax(Constants.FlyWheel2, MotorType.kBrushless);
        flyWheel3 = new CANSparkMax(Constants.FlyWheel3, MotorType.kBrushless);

       flyWheelEncoder = flyWheel1.getEncoder();

      if(Constants.RESET_SPARKMAX){
         flyWheel1.restoreFactoryDefaults();
         flyWheel2.restoreFactoryDefaults();
         if(Constants.isFlyWheel3){
            flyWheel3.restoreFactoryDefaults();
         }
      }
      
       flyWheel2.follow(flyWheel1,true); //sets inverted to true

      if(Constants.isFlyWheel3){
          flyWheel3.follow(flyWheel1);
      }
      flyWheel1.enableVoltageCompensation(12.0);
      flyWheel2.enableVoltageCompensation(12.0);

      flyWheel1.setSmartCurrentLimit(60);
      flyWheel2.setSmartCurrentLimit(60);

      if(Constants.isFlyWheel3){
         flyWheel3.enableVoltageCompensation(12.0);
         flyWheel3.setSmartCurrentLimit(60);
      }

      flyWheel1.setCANTimeout(0);
      flyWheel2.setCANTimeout(0);

      if(Constants.isFlyWheel3)
      {
         flyWheel3.setCANTimeout(0);
      }
      if (Constants.RESET_SPARKMAX) 
      {
         flyWheel1.burnFlash();
         flyWheel2.burnFlash();
         if(Constants.isFlyWheel3){
            flyWheel3.burnFlash();
         }
       }
    }


    public void shoot(double flyWheelSpeed) 
    {
       flyWheel1.set(flyWheelSpeed);
    }    

    public void notShoot(double FlyWheelSpeed) 
    {
       flyWheel1.set(FlyWheelSpeed);
    }  

    public void stop() 
    {
       flyWheel1.set(0);
    }






}
