// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Feeder;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // private Drivetrain m_drivetrain = new Drivetrain();
  // private Feeder m_feeder = new Feeder();
  // private Intake m_intake = new Intake();
  // private Shooter m_shooter = new Shooter();
  // private Climber m_climber = new Climber();

  private Joystick LEFT = new Joystick(Constants.LEFT_JOYSTICK);
  private Joystick RIGHT = new Joystick(Constants.RIGHT_JOYSTICK);

  private JoystickButton toggleLimelight, runShooter, runShooterPID, runIntake, runFeeder, raiseClimber, lowerClimber, ejectBall;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    // m_drivetrain.setDefaultCommand(
    //   new RunCommand(() -> m_drivetrain.tankDrive(LEFT.getRawAxis(Constants.YAXIS), RIGHT.getRawAxis(Constants.YAXIS)), m_drivetrain)
    // );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // raiseClimber = new JoystickButton(RIGHT, Constants.LEFT_BUTTON);
    // raiseClimber.whileHeld(new StartEndCommand(() -> m_climber.runClimber(0.5), () -> m_climber.runClimber(0), m_climber));

    // lowerClimber = new JoystickButton(RIGHT, Constants.RIGHT_BUTTON);
    // lowerClimber.whileHeld(new StartEndCommand(() -> m_climber.runClimber(-0.5), () -> m_climber.runClimber(0), m_climber));
    
    // runIntake = new JoystickButton(RIGHT, Constants.TRIGGER_BUTTON);
    // runIntake.whileHeld(new StartEndCommand(() -> m_intake.runIntake(.5, true), () -> m_intake.runIntake(0, false), m_intake));

    // runFeeder = new JoystickButton(RIGHT, Constants.MIDDLE_BUTTON);
    // runFeeder.toggleWhenPressed(new StartEndCommand(()-> m_feeder.runFeeder(.5), ()-> m_feeder.runFeeder(0), m_feeder));

    // runShooter = new JoystickButton(LEFT, Constants.TRIGGER_BUTTON);
    // runShooter.toggleWhenPressed(new StartEndCommand(() -> m_shooter.runShooter(.5, .75), () -> m_shooter.runShooter(0, 0), m_shooter));

    // runShooterPID = new JoystickButton(LEFT, Constants.TRIGGER_BUTTON);
    // runShooterPID.toggleWhenPressed(new ParallelCommandGroup(
    //   new StartEndCommand(() -> m_shooter.runTop(), () -> m_shooter.runShooter(0, 0), m_shooter), 
    //   new StartEndCommand(() -> m_shooter.runBottom(), () -> m_shooter.runShooter(0, 0), m_shooter)
    // ));

    // toggleLimelight = new JoystickButton(LEFT, Constants.LEFT_BUTTON);
    // toggleLimelight.toggleWhenPressed(new StartEndCommand(()-> m_shooter.toggleLimelightLight(3), ()-> m_shooter.toggleLimelightLight(1), m_shooter));

    // toggleLimelight = new JoystickButton(LEFT, Constants.RIGHT_BUTTON);
    // toggleLimelight.toggleWhenPressed(new StartEndCommand(()-> m_shooter.toggleLimelightCamMode(0), ()-> m_shooter.toggleLimelightLight(1), m_shooter));

    // ejectBall = new JoystickButton(LEFT, Constants.MIDDLE_BUTTON);
    // ejectBall.whileHeld(new ParallelCommandGroup(
    //   new StartEndCommand(() -> m_feeder.runFeeder(-0.5), () -> m_feeder.runFeeder(0), m_feeder), 
    //   new StartEndCommand(() -> m_intake.runIntake(-0.5, true), () -> m_intake.runIntake(0, false), m_intake)
    // ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}