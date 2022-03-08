package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;



/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written
 * explicitly for pedagogical purposes. Actual code should inline a command this
 * simple with {@link edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class DrivetrainAuto extends CommandBase {
  // The subsystem the command runs on
  private  Drivetrain driveTrain;
  private boolean complete = false;
  private double driveDist = Constants.kAutoDistance;

  public DrivetrainAuto(Drivetrain subsystem, double distance) {
    SmartDashboard.putNumber("Drive Distance", Constants.kAutoDistance);
    driveDist = distance;
    driveTrain = subsystem;
    addRequirements(driveTrain);
  }
  
  public void initialize() {

  }

  public void execute() {
    complete =  driveTrain.specificDrive(driveDist);
  }

  public void end(boolean interrupted) {
    new InstantCommand(driveTrain::stop, driveTrain);

  }

  
  public boolean isFinished() {

    return complete;
  }
}