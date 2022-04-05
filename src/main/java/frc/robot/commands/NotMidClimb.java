package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class NotMidClimb extends SequentialCommandGroup{
    public NotMidClimb(Climber m_climber) {
    addCommands(
        new InstantCommand(() -> m_climber.raiseClimber(Constants.kClimberSpeed), m_climber),
        new WaitCommand(1.1),
        new InstantCommand(() -> m_climber.raiseClimber(Constants.kClimberHold), m_climber)
    );
}
}