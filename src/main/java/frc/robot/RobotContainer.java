// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TurnToTarget;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer{

  

  // The robot's subsystems and commands are defined here...
  private Shooter m_shooter = new Shooter();
  private Intake m_intake = new Intake();
  private Drivetrain m_drive = new Drivetrain();
  private Feeder m_feeder = new Feeder();
  private Climber m_climb = new Climber();



  public Joystick left = new Joystick(0);
  public Joystick right = new Joystick(1);

  public JoystickButton shoot, toggleArm, toggleIntake, runIntake, runFeeder, climbUp, climbDown, turnToTarget;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(
      new RunCommand(
        ()-> m_drive.tankDrive(-left.getRawAxis(1), right.getRawAxis(1)), m_drive)
    ); 

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // shoot = new JoystickButton(left, 1);
    // shoot.whileHeld(new ParallelCommandGroup(new InstantCommand(()->m_limelight.ledMode.setNumber(3)), new InstantCommand(Shoot(m_shooter)), new InstantCommand(()->m_lime.pipeline.setNumber(1)));

    shoot = new JoystickButton(left, 1);
    shoot.whileHeld(new StartEndCommand(() -> m_shooter.runShooter(.5, .75),() -> m_shooter.runShooter(0, 0), m_shooter));

    runFeeder = new JoystickButton(left, 2);
    runFeeder.whileHeld(new StartEndCommand(() -> m_feeder.runFeeder(.7),() -> m_feeder.runFeeder(0), m_feeder));

    runIntake = new JoystickButton(right, 1);
    runIntake.whileHeld(new StartEndCommand(() -> m_intake.run(1),() -> m_intake.run(0), m_intake));

    //Confused on the implementation of the startendcommand
    climbUp = new JoystickButton(right, 2);
    climbUp.whileHeld(new StartEndCommand(() -> m_climb.runClimber(-.4), () -> m_climb.runClimber(0), m_climb));
    

    //climbDown = new JoystickButton(right, 2);
    //climbDown.whileHeld(new StartEndCommand(() -> m_climb.runClimber(-.3),() -> m_climb.runClimber(0), m_climb));

    //turnToTarget = new JoystickButton(right, 1);
    //turnToTarget.whileHeld(new TurnToTarget(m_drive));

    toggleArm = new JoystickButton(right, 3);
    toggleArm.whenPressed(new InstantCommand(() -> m_climb.togglePistons(), m_climb));

    toggleIntake = new JoystickButton(right, 4);
    toggleIntake.whenPressed(new InstantCommand(()->m_intake.toggleSolenoid(), m_intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(){
    // An ExampleCommand will run in autonomous
    return null;
  }
}
