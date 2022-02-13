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
import frc.robot.subsystems.Indexer;
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
<<<<<<< HEAD
  private Shooter m_shooter = new Shooter();
=======
  private Drivetrain m_drive = new Drivetrain();
  private Indexer m_feeder = new Indexer();

>>>>>>> 9c2affeccdc1908331b9d1f9beb40e8862768d05

  public Joystick left = new Joystick(1);
  public Joystick right = new Joystick(2);

  // /public static double leftSpeed, rightSpeed;

<<<<<<< HEAD
  public JoystickButton toggleIntakeSolenoid, runIntake, runShooter, run;
=======
  public JoystickButton shoot, toggleIntake, runFeeder;
>>>>>>> 9c2affeccdc1908331b9d1f9beb40e8862768d05

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // leftSpeed = left.getRawAxis(0);
    // rightSpeed = right.getRawAxis(1);
    
    m_drivetrain.setDefaultCommand(
      new RunCommand(() -> m_drivetrain.tankDrive(  left.getRawAxis(0), right.getRawAxis(1) ), m_drivetrain)
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
    runShooter = new JoystickButton(left, 1);
    runShooter.toggleWhenPressed(new StartEndCommand(() -> m_shooter.run(.5, .75), () -> m_shooter.run(0, 0), m_shooter));

<<<<<<< HEAD
    //right trigger
    runIntake = new JoystickButton(right, 1);
    runIntake.toggleWhenPressed(new StartEndCommand(() -> m_intake.set(.5), () -> m_intake.set(0), m_intake));

    //Middle button on the top of the left joystick
    toggleIntakeSolenoid = new JoystickButton(left, 2);
    toggleIntakeSolenoid.toggleWhenPressed(new StartEndCommand(() -> m_intake.setSolenoid(true), () -> m_intake.setSolenoid(false), m_intake));
=======
    // shoot = new JoystickButton(left, 1);
    // shoot.whileHeld(new ParallelCommandGroup(new InstantCommand(()->m_limelight.ledMode.setNumber(3)), new InstantCommand(Shoot(m_shooter)), new InstantCommand(()->m_lime.pipeline.setNumber(1)));


    toggleIntake = new JoystickButton(right, 1);
    toggleIntake.whileHeld(
      new StartEndCommand(
        () -> m_intake.setIntake(true, .7), 
        () -> m_intake.setIntake(false, 0), 
        m_intake));

    shoot = new JoystickButton(left, 1);
    shoot.whileHeld(new InstantCommand(() -> m_shooter.runShooter(), m_shooter));

    runFeeder = new JoystickButton(left, 2);
    runFeeder.whileHeld(new InstantCommand(() -> m_feeder.runFeeder(), m_feeder));
>>>>>>> 9c2affeccdc1908331b9d1f9beb40e8862768d05
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