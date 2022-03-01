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
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.AimAndShoot;
import frc.robot.commands.DrivetrainAuto;
import frc.robot.commands.ShootAndDriveAuto;
import frc.robot.commands.TurnToTarget;

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
    private final Intake m_innerIntake = new Intake();
  private final FlyWheel m_flyWheel = new FlyWheel();
  private final Limelight m_limelight = new Limelight();
  private final Constants m_constants= new Constants();
  private final XboxController controller = new XboxController(1);
  private final Climber m_climber = new Climber();
  private final Joystick joystick = new Joystick(2);
  //private frc.robot.commands.AimAndShoot AimAndShoot = new AimAndShoot(m_drivetrain, m_flyWheel, m_conveyor);
 // private final XboxController controller2 = new XboxController(2);
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  ;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_drivetrain.setDefaultCommand(
      new RunCommand(() -> m_drivetrain.drivePercentController(controller,Constants.maxSpeed),m_drivetrain));

      m_chooser.setDefaultOption("Shoot Then Drive", new ShootAndDriveAuto(m_drivetrain,m_flyWheel,m_conveyor,m_innerIntake));
      m_chooser.addOption("Drive Only",new DrivetrainAuto(m_drivetrain, Constants.kAutoDistance));
      SmartDashboard.putData("Auto Chooser: ", m_chooser);

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
    //inner intake forwward rb
    new JoystickButton(controller, XboxController.Button.kRightBumper.value)
    .whenHeld(new InstantCommand(() -> m_innerIntake.intakeReverse(Constants.kIntakeHigh), m_innerIntake))
         .whenReleased(new InstantCommand(m_innerIntake::intakeStop, m_innerIntake));
   
      //move elevator (conveyer) UP lb
    new JoystickButton(controller, XboxController.Button.kLeftBumper.value)
    .whenHeld(new InstantCommand(() -> m_innerIntake.intakeReverse(Constants.kIntakeHigh), m_innerIntake))
    .whenHeld(new InstantCommand(() -> m_conveyor.raiseConveyor(Constants.kConveyerHigh), m_conveyor))
    .whenReleased(new InstantCommand(() -> m_conveyor.stopConveyor(), m_conveyor))
      .whenReleased(new InstantCommand(m_innerIntake::intakeStop, m_innerIntake));
   
      //flywheel shoot fast y
    new JoystickButton(controller, XboxController.Button.kY.value)
    .whenHeld(new InstantCommand(() -> m_climber.raiseClimber(Constants.kClimberBack), m_climber))
      .whenReleased(new InstantCommand(m_climber::stopClimber, m_climber));
    
   
      //Lower convyeyor b
    new JoystickButton(controller, XboxController.Button.kB.value)
    .whenHeld(new InstantCommand(() -> m_conveyor.lowerConveyor(Constants.kConveyorLow), m_conveyor))
    .whenHeld(new InstantCommand(() -> m_innerIntake.intakeForward(Constants.kIntakeLow), m_innerIntake))
    .whenHeld(new InstantCommand(() -> m_flyWheel.shoot(.75), m_flyWheel))
    .whenReleased(new InstantCommand(() ->  m_flyWheel.stop(), m_flyWheel))
    .whenReleased(new InstantCommand(() ->  m_innerIntake.intakeStop(), m_innerIntake))
      .whenReleased(new InstantCommand(m_conveyor::stopConveyor, m_conveyor));
   
   
      //deploy intake right stick
   // new JoystickButton(controller, XboxController.Button.kRightStick.value)
      //.whenPressed(new InstantCommand(() -> m_intake.deployIntake(), m_innerIntake))
     //   .whenReleased(new InstantCommand(m_intake::off, m_intake));
   
        //retract intake left stick
      // new JoystickButton(controller, XboxController.Button.kLeftStick.value)
     // .whenPressed(new InstantCommand(() -> m_intake.retractIntake(),m_innerIntake))
       // .whenReleased(new InstantCommand(m_intake::off,m_intake));

        //Shoot slow x
    new JoystickButton(controller, XboxController.Button.kX.value)
    .whenPressed(new InstantCommand(() -> m_climber.lowClimb(Constants.kClimberBack), m_climber))
        .whenReleased(new InstantCommand(m_climber::stopClimber,m_climber));
   
        //intake out start
   // new JoystickButton(controller, XboxController.Button.kStart.value)
     // .whenPressed(new InstantCommand(() -> m_intake.intakeOut(Constants.kIntakeHigh), m_innerIntake))
       // .whenReleased(new InstantCommand(m_intake::intakeStop, m_intake));
       //new JoystickButton(controller, XboxController.Button.kA.value)
       // .whenPressed(new TurnToTarget(m_drivetrain));
       new JoystickButton(controller, XboxController.Button.kA.value)
       .whenPressed( new TurnToTarget(m_drivetrain));

         new JoystickButton(controller, XboxController.Button.kBack.value)
         .whenPressed(new InstantCommand(m_limelight::enableLED,m_limelight));

         new JoystickButton(controller, XboxController.Button.kStart.value)
         .whenPressed(new InstantCommand(m_limelight::disableLED,m_limelight));

         new JoystickButton(controller, XboxController.Button.kRightStick.value)
         .whenPressed(new InstantCommand(m_limelight::findRing,m_limelight));



//JOYCON

//RB - Set coast mode to climber
        new JoystickButton(joystick, Constants.kRB)
         .whenPressed(new InstantCommand(() -> m_climber.setCoast(),m_climber));
// RT - Set break mode to climber
         new JoystickButton(joystick, Constants.kRT)
         .whenPressed(new InstantCommand(() -> m_climber.setBrake(),m_climber));

  //flywheel shoot fast Y
  new JoystickButton(joystick, Constants.kY)
  .whenHeld(new InstantCommand(() -> m_flyWheel.shoot(Constants.kFlyWheelFast), m_flyWheel))
    .whenReleased(new InstantCommand(m_flyWheel::stop, m_flyWheel));

  //flywheel shoot fast X
  new JoystickButton(joystick, Constants.kX)
  .whenHeld(new InstantCommand(() -> m_flyWheel.shoot(Constants.kFlyWheelSlow), m_flyWheel))
    .whenReleased(new InstantCommand(m_flyWheel::stop, m_flyWheel));

//Lt- lower climber
         new JoystickButton(joystick, Constants.kLT)
         .whenPressed(new InstantCommand(() -> m_climber.simpleFrontLower(Constants.kClimberSpeed),m_climber))
         .whenReleased(new InstantCommand(m_climber::stopClimber,m_climber));

//LB - CLIMB = CRUNCH!!        
         new JoystickButton(joystick, Constants.kLB)
         .whenPressed(new InstantCommand(() -> m_climber.simpleFrontRaise(Constants.kClimberSpeed),m_climber))
         .whenReleased(new InstantCommand(m_climber::stopClimber,m_climber));

    
         
// Joycon A - Lower conveyor and flywheel
         new JoystickButton(joystick, Constants.kA)
         .whenHeld(new InstantCommand(() -> m_conveyor.lowerConveyor(Constants.kConveyorLow), m_conveyor))
        .whenHeld(new InstantCommand(() -> m_flyWheel.notShoot(Constants.kFlyWheelSlow), m_flyWheel))
       .whenReleased(new InstantCommand(() ->  m_flyWheel.stop(), m_flyWheel))
        .whenReleased(new InstantCommand(m_conveyor::stopConveyor, m_conveyor));


         //new JoystickButton(controller, XboxController.Button.kRightStick.value)
         //.whenHeld(new InstantCommand(() -> m_climber.lowerClimber(.75),m_climber)).whenReleased(new InstantCommand(m_climber::stopClimber,m_climber));
    }
    public Command getAutonomousCommand(){
     // return new DrivetrainAuto(m_drivetrain, Constants.kAutoDistance);
     return m_chooser.getSelected();

    }
  }





