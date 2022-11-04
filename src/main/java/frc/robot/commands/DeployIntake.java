package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FancyIntake;
import frc.robot.Constants;
public class DeployIntake extends CommandBase{
    private FancyIntake m_fancyIntake;



@Override
public void initialize() 
{

}

@Override
public void execute() {

        m_fancyIntake.deployOuter(Constants.kIntakeLow);
    
}

@Override
public void end(boolean interrupted) {
    m_fancyIntake.halt();
}

@Override
public boolean isFinished() {
    return m_fancyIntake.getToggleTop();
}

}
