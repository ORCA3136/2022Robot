package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
 

public class FlyWheel extends SubsystemBase {



    private CANSparkMax FlyWheel1;
    private  CANSparkMax FlyWheel2;
    private CANSparkMax FlyWheel3;
    RelativeEncoder FlyWheelEncoder;


    public FlyWheel(){
        FlyWheel1 = new CANSparkMax(Constants.FlyWheel1, MotorType.kBrushless);
        FlyWheel2 = new CANSparkMax(Constants.FlyWheel2, MotorType.kBrushless);
        FlyWheel3 = new CANSparkMax(Constants.FlyWheel3, MotorType.kBrushless);

         FlyWheelEncoder = FlyWheel1.getEncoder();

      if(Constants.RESET_SPARKMAX){
         FlyWheel1.restoreFactoryDefaults();
         FlyWheel2.restoreFactoryDefaults();
         if(Constants.isFlyWheel3){
            FlyWheel3.restoreFactoryDefaults();
         }
      }
      
      FlyWheel2.follow(FlyWheel1);
      if(Constants.isFlyWheel3){
         FlyWheel3.follow(FlyWheel1);
      }
      FlyWheel1.enableVoltageCompensation(12.0);

      FlyWheel1.setSmartCurrentLimit(60);
      FlyWheel2.setSmartCurrentLimit(60);
      if(Constants.isFlyWheel3){
         FlyWheel3.setSmartCurrentLimit(60);
      }

      FlyWheel1.setCANTimeout(0);
      FlyWheel2.setCANTimeout(0);
      if(Constants.isFlyWheel3){
         FlyWheel3.setCANTimeout(0);
      }
      if (Constants.RESET_SPARKMAX) {
         FlyWheel1.burnFlash();
         FlyWheel2.burnFlash();
         if(Constants.isFlyWheel3){
            FlyWheel3.burnFlash();
         }
       }
    }
    public void Shoot(double FlyWheelSpeed) 
    {FlyWheel1.set(FlyWheelSpeed);}    

    public void NotShoot(double FlyWheelSpeed) 
    {FlyWheel1.set(FlyWheelSpeed);}  

    public void Stop() 
    {FlyWheel2.set(0);}  

}
