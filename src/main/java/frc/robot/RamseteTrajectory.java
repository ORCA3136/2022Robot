package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;
import java.nio.file.Path;
import java.io.IOException;

public class RamseteTrajectory {
    private Drivetrain m_drivetrain;
    private DifferentialDriveVoltageConstraint voltageConstraint;
    private SimpleMotorFeedforward ramseteFF; 

    public PathPlannerTrajectory loadPath(String NewNewPath, double Eight, double Five){}

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, 
        Constants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(voltageConstraint);

    public void Ramsete(Drivetrain drivetrain) {
        this.m_drivetrain = drivetrain;
    
        ramseteFF = new SimpleMotorFeedforward(
          Constants.ksVolts,
          Constants.kvVoltSecondsPerMeter,
          Constants.kaVoltSecondsSquaredPerMeter);
    
        voltageConstraint = new DifferentialDriveVoltageConstraint(
          ramseteFF,
          Constants.DRIVE_KINEMATICS,
          Constants.MAX_VOLTAGE);
      }

    public Trajectory getTrajectory() {
        Trajectory threeBall = PathPlanner.loadPath("New New Path", 8, 5);
        Trajectory.State state = threeBall.sample(1.2);
        System.out.println(state.velocityMetersPerSecond);
    };

    RamseteCommand ramseteCommand = new RamseteCommand(
            getTrajectory(),
            m_drivetrain::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            ramseteFF,
            new SimpleMotorFeedforward(Constants.ksVolts,
            Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            m_drivetrain::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0)   ,  
            new PIDController(Constants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_drivetrain::tankDriveVolts,
            m_drivetrain
        );


        //m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
}
