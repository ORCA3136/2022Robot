package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.FancyIntake;
import frc.robot.subsystems.Intake;

public class ShootAndDriveAuto extends SequentialCommandGroup
{
    public ShootAndDriveAuto(Drivetrain driveTrain, FlyWheel flyWheel, Conveyor conveyor, Intake intake, FancyIntake fancyIntake)
    {
        addCommands(
            new InstantCommand(() -> fancyIntake.servoHand(), fancyIntake),
            //start the flywheel
            new InstantCommand(() -> flyWheel.PIDshoot(Constants.kShooterHighTargetAutoTest, Constants.kShooterHighTargetF3AutoRPS), flyWheel),
            //wait for spinup
            new WaitCommand(2),//TODO manage flywheel target speed at somepoint
            //start conveyor and intake
            new InstantCommand(() -> intake.intakeIn(Constants.kIntakeHigh), intake).andThen(
            new InstantCommand(() -> conveyor.raiseConveyor(Constants.kConveyerHigh), conveyor)),
            new WaitCommand(1.5),
            new InstantCommand(() ->  flyWheel.stop(), flyWheel),
            new InstantCommand(() -> conveyor.stopConveyor(), conveyor),
            //driveforward to get the next ball
            new InstantCommand(intake::intakeStop, intake),
            //drive forward..and run the intake
            new InstantCommand(() -> fancyIntake.wheelsIn(Constants.kIntakeLow), fancyIntake),
            new InstantCommand(() -> intake.intakeIn(Constants.kIntakeHigh), intake),
            new DrivetrainAuto(driveTrain, Constants.kAutoDistance),
            new InstantCommand(() -> conveyor.lowerConveyor(Constants.kConveyorLow), conveyor),
            new InstantCommand(()-> flyWheel.notShootAuto(Constants.kFlyWheelFast),flyWheel),
            new WaitCommand(.5),
            new InstantCommand(() -> conveyor.stopConveyor(), conveyor),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(driveTrain::stop, driveTrain),
            new InstantCommand(() ->  flyWheel.stop(), flyWheel),
            //drive back to shooting range....TODO move to a limelight calibrated distance
            new DrivetrainAutoReverse(driveTrain, Constants.kAutoDistance),
            new InstantCommand(()-> flyWheel.PIDshoot(Constants.kShooterHighTargetRPS, Constants.kShooterHighTargetF3RPS), flyWheel),
          /*  new TurnToTarget(driveTrain),
            new TurnToTarget(driveTrain), 
            new TurnToTarget(driveTrain),
            new TurnToTarget(driveTrain), 
            new TurnToTarget(driveTrain),
            new TurnToTarget(driveTrain), 
           */
            new WaitCommand(2),
            new InstantCommand(() -> intake.intakeIn(Constants.kIntakeHigh), intake)
            .andThen(new InstantCommand(() -> conveyor.raiseConveyor(Constants.kConveyerHigh), conveyor)),
            new WaitCommand(1),
            new InstantCommand(() ->  flyWheel.stop(), flyWheel),
            new InstantCommand(() -> conveyor.stopConveyor(), conveyor),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> fancyIntake.wheelsStop(), fancyIntake)
                        );
    }

}