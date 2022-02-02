package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;    
import frc.robot.Constants;


public class Conveyor extends SubsystemBase{
    private CANSparkMax leftConveyor;
    private CANSparkMax rightConveyor;



public Conveyor(){
    leftConveyor = new CANSparkMax(Constants.kConveyor1, MotorType.kBrushless);
    rightConveyor.follow(leftConveyor, true);
}

public void lowerConveyor(){
    leftConveyor.set( Constants.kConveyorSpeed *-1 );
}

public void raiseConveyor(){
    leftConveyor.set( Constants.kConveyorSpeed);
}

public void stopConveyor(){
    leftConveyor.set(0);
}



}
