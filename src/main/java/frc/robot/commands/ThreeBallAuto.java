package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;

public class ThreeBallAuto extends SequentialCommandGroup
{
    public ThreeBallAuto(Drivetrain driveTrain, FlyWheel flyWheel, Conveyor conveyor, Intake intake)
    {
        addCommands(
            //start the flywheel
            new InstantCommand(() -> flyWheel.PIDshoot(Constants.kShooterHighTargetRPS, Constants.kShooterHighTargetF3RPS), flyWheel),
            //wait for spinup
            new WaitCommand(.9),
            //start conveyor and intake
            new InstantCommand(() -> intake.intakeIn(Constants.kIntakeHigh), intake).andThen(
            new InstantCommand(() -> conveyor.raiseConveyor(Constants.kConveyerHigh), conveyor)),
            new WaitCommand(.9),
            new InstantCommand(() ->  flyWheel.stop(), flyWheel),
            new InstantCommand(() -> conveyor.stopConveyor(), conveyor),
            //driveforward to get the next ball
            new InstantCommand(intake::intakeStop, intake),
            //drive forward..and run the intake
            new InstantCommand(() -> intake.intakeIn(Constants.kIntakeHigh), intake),
            new DrivetrainAuto(driveTrain, Constants.kAutoDistance),
            new InstantCommand(() -> intake.intakeOut(Constants.kIntakeLow), intake),            
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
            new TurnToTarget(driveTrain),
            new TurnToTarget(driveTrain), 
            new TurnToTarget(driveTrain),
            new TurnToTarget(driveTrain), 
            new TurnToTarget(driveTrain),
            new TurnToTarget(driveTrain), 
            new WaitCommand(2),
            new InstantCommand(() -> intake.intakeIn(Constants.kIntakeHigh), intake)
            .andThen(new InstantCommand(() -> conveyor.raiseConveyor(Constants.kConveyerHigh), conveyor)),
            new WaitCommand(1),
            //goes back to where the ball was, turns, and drives distance
            new DrivetrainAuto(driveTrain, Constants.kAutoDistance),
            new DriveTrainAutoTurn(driveTrain, Constants.kAutoDistanceTurn),
            new InstantCommand(() -> intake.intakeIn(Constants.kIntakeHigh), intake),
            new DrivetrainAuto(driveTrain, Constants.kAutoDistanceThree),
            //runs intake to bring it in and pushes it off flywheel
            new InstantCommand(() -> intake.intakeOut(Constants.kIntakeLow), intake),            
            new InstantCommand(() -> conveyor.lowerConveyor(Constants.kConveyorLow), conveyor),
            new InstantCommand(()-> flyWheel.notShootAuto(Constants.kFlyWheelFast),flyWheel),
            new WaitCommand(.5),
            //stops everything
            new InstantCommand(() -> conveyor.stopConveyor(), conveyor),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(driveTrain::stop, driveTrain),
            new InstantCommand(() ->  flyWheel.stop(), flyWheel),
            //runs flywheel and turns to target
            new InstantCommand(()-> flyWheel.PIDshoot(Constants.kShooterHighTargetRPS, Constants.kShooterHighTargetF3RPS), flyWheel),
            new TurnToTarget(driveTrain),
            new TurnToTarget(driveTrain), 
            new TurnToTarget(driveTrain),
            new TurnToTarget(driveTrain),   
            new TurnToTarget(driveTrain),
            new TurnToTarget(driveTrain), 
            new WaitCommand(2),
            //shoots  it 
            new InstantCommand(() -> intake.intakeIn(Constants.kIntakeHigh), intake)
            .andThen(new InstantCommand(() -> conveyor.raiseConveyor(Constants.kConveyerHigh), conveyor)),
            new WaitCommand(1),
            new InstantCommand(() ->  flyWheel.stop(), flyWheel),
            new InstantCommand(() -> conveyor.stopConveyor(), conveyor),
            new InstantCommand(intake::intakeStop, intake)
            );  
    }

}