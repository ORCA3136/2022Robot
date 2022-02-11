// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.InnerIntake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain(); 
  private final Conveyor m_conveyor= new Conveyor();
  private final Intake m_intake =  new Intake();
  private final InnerIntake m_innerIntake = new InnerIntake();
  private final FlyWheel m_flyWheel = new FlyWheel();
  private final Constants m_constants= new Constants();
  private final XboxController controller = new XboxController(1);
  private final Joystick Joystick = new Joystick(0);
  public void Constants(){
 
  };

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_drivetrain.setDefaultCommand(
      new RunCommand(() -> m_drivetrain.drive(controller),m_drivetrain));


    // Configure the button bindings
    
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    // An ExampleCommand will run in autonomous
    configureButtonBindings();


  }


  
  private void configureButtonBindings() {

    new JoystickButton(controller, XboxController.Button.kRightBumper.value)
    .whenHeld(new InstantCommand(() -> m_innerIntake.intakeForward(Constants.kIntakeHigh), m_innerIntake))
      .whenReleased(new InstantCommand(m_innerIntake::intakeStop, m_innerIntake));
  


}


  }





