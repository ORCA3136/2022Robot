package frc.robot.commands;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;

public class ShootAndDriveAuto extends SequentialCommandGroup
{
    public ShootAndDriveAuto(Drivetrain driveTrain, FlyWheel flyWheel, Conveyor conveyor, Intake intake)
    {
        addCommands(
            //start the flywheel
            new InstantCommand(() -> flyWheel.shoot(Constants.kFlyWheelAuto), flyWheel),
            //wait for spinup
            new WaitCommand(2),//TODO manage flywheel target speed at somepoint
            //start conveyor and intake
            new InstantCommand(() -> intake.intakeReverse(Constants.kIntakeHigh), intake).andThen(
            new InstantCommand(() -> conveyor.raiseConveyor(Constants.kConveyerHigh), conveyor)),
            //new WaitCommand(1.5),
            new InstantCommand(() ->  flyWheel.stop(), flyWheel),
            new InstantCommand(() -> conveyor.stopConveyor(), conveyor),
            //driveforward to get the next ball
            new InstantCommand(intake::intakeStop, intake),
            //drive forward...
            new InstantCommand(() -> intake.intakeReverse(Constants.kIntakeHigh), intake),
            new DrivetrainAuto(driveTrain, Constants.kAutoDistance),
            new InstantCommand(intake::intakeStop, intake),

            new InstantCommand(()-> flyWheel.notShoot(Constants.kFlyWheelFast),flyWheel),
            new WaitCommand(.65),
            new PrintCommand("Completed Drive Auto Command"),
            new InstantCommand(driveTrain::stop, driveTrain),
            new InstantCommand(() ->  flyWheel.stop(), flyWheel),
            new WaitCommand(2),
            new InstantCommand(()-> flyWheel.shoot(-1), flyWheel),///NOTE FULL SEND RIGHT NOW FIX THIS - just drive closer
            new WaitCommand(1.5),
            new TurnToTarget(driveTrain),
            new TurnToTarget(driveTrain), 
            //new InstantCommand(() -> intake.intakeReverse(.2),intake),
            //new DrivetrainAuto(driveTrain, Constants.kAutoDistance2),
            new WaitCommand(1.5),


            new InstantCommand(() -> intake.intakeReverse(Constants.kIntakeHigh), intake)
            .andThen(new InstantCommand(() -> conveyor.raiseConveyor(Constants.kConveyerHigh), conveyor)),
            new WaitCommand(2),
            new InstantCommand(() ->  flyWheel.stop(), flyWheel),
            new InstantCommand(() -> conveyor.stopConveyor(), conveyor),
            //driveforward to get the next ball
            new InstantCommand(intake::intakeStop, intake),
            //new DrivetrainAuto(driveTrain, Constants.kAutoDistance2),
            new PrintCommand("Drive Auto 2")
            
            //stop conveyor and intake
            //new InstantCommand(driveTrain::stop, driveTrain));
            );
    }

}