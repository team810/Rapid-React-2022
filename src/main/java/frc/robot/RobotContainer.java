// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
  // The robot's subsystems and commands are defined here...
  // private Drivetrain m_drivetrain = new Drivetrain();
  // private Intake m_intake = new Intake();
  private Shooter m_shooter = new Shooter();

  public Joystick left = new Joystick(1);
  public Joystick right = new Joystick(2);

  public JoystickButton toggleIntakeSolenoid, runIntake, runShooter;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    // Configure the button bindings
    configureButtonBindings();

    // m_drivetrain.setDefaultCommand(
    //   new RunCommand(()-> m_drivetrain.tankDrive(left.getRawAxis(0), right.getRawAxis(1)), m_drivetrain)
    // );
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() 
  {
    runShooter  = new JoystickButton(left, 1);
    runShooter.toggleWhenPressed(new StartEndCommand(()-> m_shooter.run(.5), ()-> m_shooter.run(0), m_shooter));

    // runIntake = new JoystickButton(left, 1);
    // runIntake.toggleWhenPressed(new StartEndCommand(()-> m_intake.set(1),()-> m_intake.set(0), m_intake));

    // toggleIntakeSolenoid = new JoystickButton(left, 2);
    // toggleIntakeSolenoid.toggleWhenPressed(new StartEndCommand(()-> m_intake.setSolenoid(true),()-> m_intake.setSolenoid(false), m_intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() 
  {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
