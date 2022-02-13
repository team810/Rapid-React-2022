// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

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
  private Drivetrain m_drivetrain = new Drivetrain();
  private Intake m_intake = new Intake();
  private Shooter m_shooter = new Shooter();

  public Joystick LEFT = new Joystick(1);
  public Joystick RIGHT = new Joystick(2);

  public JoystickButton runIntake, runShooter, runFeeder;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    m_drivetrain.setDefaultCommand(
      new RunCommand(() -> m_drivetrain.tankDrive(LEFT.getRawAxis(0), RIGHT.getRawAxis(1)), m_drivetrain)
    );
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
    // Left trigger
    runShooter = new JoystickButton(LEFT, 1);
    runShooter.toggleWhenPressed(new StartEndCommand(() -> m_shooter.runShooter(.5, .75), () -> m_shooter.runShooter(0, 0), m_shooter));

    //right trigger
    runIntake = new JoystickButton(RIGHT, 1);
    runIntake.toggleWhenPressed(new StartEndCommand(() -> m_intake.runIntake(.5, true), () -> m_intake.runIntake(0, false), m_intake));

    //Middle button left joystick
    runFeeder = new JoystickButton(LEFT, 2);
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