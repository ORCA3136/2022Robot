package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class DriveOnlyAuto extends SequentialCommandGroup
{
    public DriveOnlyAuto(Drivetrain driveTrain)
    {
        addCommands(
            new DrivetrainAuto(driveTrain),
            new PrintCommand("Completed Drive Auto Command"),
            new InstantCommand(driveTrain::stop, driveTrain)
        );
    }

}