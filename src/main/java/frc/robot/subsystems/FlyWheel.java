package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
 

public class FlyWheel extends SubsystemBase {



    private CANSparkMax FlyWheelRight;
    private static CANSparkMax FlyWheelLeft;

    public FlyWheel(){
        FlyWheelRight = new CANSparkMax(Constants.RightFlyWheel, MotorType.kBrushless);
        FlyWheelLeft = new CANSparkMax(Constants.LeftFlyWheel, MotorType.kBrushless);
    }

    public void Shoot( XboxController controller) 
    {

       FlyWheelLeft.set(Constants.kFlyWheelSpeed);
       FlyWheelRight.set(Constants.kFlyWheelSpeed * -1);

    }   

    public void NotShoot( XboxController controller) 
    {

       FlyWheelLeft.set(Constants.kFlyWheelSpeed * -1);
       FlyWheelRight.set(Constants.kFlyWheelSpeed);

    }  

}
