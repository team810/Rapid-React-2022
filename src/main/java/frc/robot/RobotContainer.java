// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ShootSequence;
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
  // SUBSYSTEMS
  public Intake m_intake = new Intake();
  public Drivetrain m_drive = new Drivetrain();
  public Feeder m_feeder = new Feeder();
  public Shooter m_shooter = new Shooter();
  private Climber m_climb = new Climber();
  private Limelight m_lime = new Limelight();

  //CONTROLLERS
  public Joystick left = new Joystick(0);
  public Joystick right = new Joystick(1);
  public Joystick gamepad = new Joystick(2);

  //AUTONOMOUS PATHS
  public HashMap<String, Trajectory> pathsTrajs = new HashMap<String, Trajectory>();
  public HashMap<String, Command> paths = new HashMap<String, Command>();
  public String[] trajNames = {"Simple Blue Auto 1", "Simple Blue Auto 2", "Simple Blue Auto 3", "Three Ball Blue",
                                "Simple Red Auto 1", "Simple Red Auto 2", "Simple Red Auto 3", "Three Ball Red"};

  //AUTONOMOUS PATH CHOOSER
  public SendableChooser<String> m_chooser = new SendableChooser<String>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //SUBSYTEM INITIAL TASKS
    m_drive.setDefaultCommand(
      new RunCommand(
        ()-> m_drive.tankDrive(Math.pow(left.getRawAxis(1), 1), Math.pow(right.getRawAxis(1), 1))
        , m_drive
    )); 

    m_intake.setIntake(false);

    // ADD PATHS TO CHOOSER
    for(String key : paths.keySet()){
      m_chooser.addOption(key, key);
    }
    // ADD CHOOSER TO SMART DASHBOARD
    SmartDashboard.putData("Chooser", m_chooser);
  }

  public double Cubic(double value, double weight){
    return weight * value * value * value - (1 - weight) * value;
  }

  public double deadBandCutoff(double value){

    final double cutoff = .1;

    final double weight = .2;

    if(Math.abs(value) < cutoff){
      return 0;
    }
    else{
      return Cubic(value, weight - Math.abs(value)/value*Cubic(cutoff, weight)) / (1.0 - Cubic(cutoff, weight));
    }
  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Shoot withOUT PID
    new JoystickButton(gamepad, 8)
    .whileHeld(new StartEndCommand(() -> m_shooter.runShooter(.3, .3),() -> m_shooter.runShooter(0, 0), m_shooter));

    //Shoot with PID
    new JoystickButton(left, 1)
      .whileHeld(new ShootSequence(m_drive, m_feeder, m_shooter));

    //Align Shooter
    new JoystickButton(left, 2)
      .whileHeld(new TurnToTarget(m_drive));
    
    //Run Intake
    new JoystickButton(gamepad, 7)
    .whileHeld(
      new ParallelCommandGroup(
        new StartEndCommand(() -> m_intake.run(1),() -> m_intake.run(0), m_intake),
        new StartEndCommand(() -> m_feeder.runFeeder(1),() -> m_feeder.runFeeder(0), m_feeder)
      )
    );

    //Run Intake
    new JoystickButton(right, 1)
    .whileHeld(
      new ParallelCommandGroup(
        new StartEndCommand(() -> m_intake.run(1),() -> m_intake.run(0), m_intake),
        new StartEndCommand(() -> m_feeder.runFeeder(1),() -> m_feeder.runFeeder(0), m_feeder)
      )
    );

    //Run Feeder
    new JoystickButton(gamepad, 5) 
      .whileHeld(new StartEndCommand(() -> m_feeder.runFeeder(1),() -> m_feeder.runFeeder(0), m_feeder));
    
    new JoystickButton(right, 2) 
      .whileHeld(new StartEndCommand(() -> m_feeder.runFeeder(1),() -> m_feeder.runFeeder(0), m_feeder));

    //Toggle intake
    new JoystickButton(right, 3)
      .whenPressed(new InstantCommand(()->m_intake.toggleSolenoid(), m_intake));

    //Reverse Intake and Feeder
    new JoystickButton(right, 4)
      .whileHeld(
        new ParallelCommandGroup(
          new StartEndCommand(() -> m_feeder.runFeeder(-1),() -> m_feeder.runFeeder(0), m_feeder),
          new StartEndCommand(()->m_intake.run(-1), ()->m_intake.run(0), m_intake)
        )
      );

    //Reset encoders
    new JoystickButton(right, 6)
      .whenPressed(new InstantCommand(()->m_drive.resetEncoders()));

    //Reset navx
    new JoystickButton(right, 7)
      .whenPressed(new InstantCommand(()->m_drive.resetGyro()));

    //Climb UP
    new JoystickButton(gamepad, 4)
      .whileHeld(
        new ParallelCommandGroup(
          new StartEndCommand(() -> m_climb.climbUp(-.6), () -> m_climb.climbUp(0), m_climb),
          new StartEndCommand(()->m_lime.ledMode.setNumber(2), ()-> m_lime.ledMode.setNumber(0), m_lime)
        )
      );
    
    //Climb DOWN
    new JoystickButton(gamepad, 2)
      .whileHeld(new StartEndCommand(() -> m_climb.climbDown(.6),() -> m_climb.climbDown(0), m_climb));

    //Toggle climber arms
    new JoystickButton(gamepad, 3)
      .whenPressed(new InstantCommand(() -> m_climb.togglePistons(), m_climb));

    //Toggle and run intake
    //new JoystickButton(GP, 9)
    //  .whileHeld(new StartEndCommand(() -> m_intake.setIntake(true), () -> m_intake.setIntake(false), m_intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command command = paths.get(trajNames[0]);
    Trajectory m_traj = pathsTrajs.get(trajNames[0]);

    m_drive.resetOdometry(m_traj.getInitialPose());

    // AUTONAV
    return new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        command,
        new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
        new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder)
      ),
      new ShootSequence(m_drive, m_feeder, m_shooter).withTimeout(5)
    ).andThen(() -> m_drive.tankDriveVolts(0, 0));
  }
}
