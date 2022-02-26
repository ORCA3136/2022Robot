package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;


/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written
 * explicitly for pedagogical purposes. Actual code should inline a command this
 * simple with {@link edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class DrivetrainAuto extends CommandBase {
  // The subsystem the command runs on
  private final Drivetrain driveTrain;

  public DrivetrainAuto(Drivetrain subsystem) {
    SmartDashboard.putNumber("Drive Distance", 10);
    driveTrain = subsystem;
    addRequirements(driveTrain);
  }

  
  public void initialize() {
    double rightDistance = SmartDashboard.getNumber("Drive Distance", 10);
    double leftDistance = SmartDashboard.getNumber("Drive Distance", 10);
    driveTrain.drivePercent(rightDistance, leftDistance);
  }

  
  public boolean isFinished() {
    return false;
  }
}