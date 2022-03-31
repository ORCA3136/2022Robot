package frc.robot;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.subsystems.Drivetrain;

public class RamseteTrajectory {
    private Drivetrain m_drivetrain;
    private DifferentialDriveVoltageConstraint voltageConstraint;
    private SimpleMotorFeedforward ramseteFF; 
  

 public enum PathPlannerPath {

  THREE_BALL(PathPlanner.loadPath("New New Path", 6, 3, true));

  PathPlannerTrajectory trajectory;

  private PathPlannerPath(PathPlannerTrajectory trajectory) {
    this.trajectory = trajectory;
  }

  public PathPlannerTrajectory getTrajectory() {
    return trajectory;
  }
}

public RamseteTrajectory(Drivetrain m_drivetrain) {
  this.m_drivetrain = m_drivetrain;

  ramseteFF = new SimpleMotorFeedforward(
    Constants.ksVolts,
    Constants.kvVoltSecondsPerMeter,
    Constants.kaVoltSecondsSquaredPerMeter);

  voltageConstraint = new DifferentialDriveVoltageConstraint(
    ramseteFF,
    Constants.DRIVE_KINEMATICS,
    Constants.MAX_VELOCITY_MPS);
}


public RamseteCommand createRamseteCommand(PathPlannerPath path) {
  return new RamseteCommand(
    path.getTrajectory(),
    m_drivetrain::getPose,
    new RamseteController(
      Constants.kRamseteB,
      Constants.kRamseteZeta),
    ramseteFF,
    Constants.kDrive.DRIVE_KINEMATICS,
    m_drivetrain::getWheelSpeeds,
    new PIDController(
      Constants.kPDriveVel,
      Constants.kIDrive,
      Constants.kDDrive),
    new PIDController(
      Constants.kPDriveVel,
      Constants.kIDrive,
      Constants.kDDrive),
    m_drivetrain::tankDriveVolts,
    m_drivetrain);
}

public DifferentialDriveVoltageConstraint getVoltageConstraint() {
  return voltageConstraint;
}

}
