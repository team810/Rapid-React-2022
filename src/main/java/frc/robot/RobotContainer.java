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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.TurnToTarget;
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
  private Intake m_intake = new Intake();
  private Drivetrain m_drive = new Drivetrain();
  private Feeder m_feeder = new Feeder();
  private Shooter m_shooter = new Shooter();
  private Climber m_climb = new Climber();
  private Limelight m_lime = new Limelight();

  //CONTROLLERS
  public Joystick left = new Joystick(0);
  public Joystick right = new Joystick(1);
  public XboxController GP = new XboxController(2);

  //BUTTONS (UNNECESSARY)
  public JoystickButton shoot, shootPID, toggleArm, toggleIntake, runIntake, runFeeder, turnToTarget, setIntake, slow, climbDown, climbUp;

  //AUTONOMOUS PATHS
  public HashMap<String, Trajectory> pathsTrajs = new HashMap<String, Trajectory>();
  public HashMap<String, Command> paths = new HashMap<String, Command>();
  public String[] trajNames = {"pathplanner/generatedJSON/FIRST.wpilib.json"};

  //AUTONOMOUS PATH CHOOSER
  public SendableChooser<String> m_chooser = new SendableChooser<String>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //SUBSYTEM INITIAL TASKS
    m_drive.setDefaultCommand(
      new RunCommand(
        ()-> m_drive.tankDrive(left.getRawAxis(1), right.getRawAxis(1)), m_drive)
    ); 
    m_intake.setIntake(false);

    //MAKE AUTO PATHS
    initPaths();
    // ADD PATHS TO CHOOSER
    for(String key : paths.keySet()){
      m_chooser.addOption(key, key);
    }
    // ADD CHOOSER TO SMART DASHBOARD
    SmartDashboard.putData("Chooser", m_chooser);
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

    //shoot = new JoystickButton(left, 3);
    //shoot.whileHeld(new StartEndCommand(() -> m_shooter.runShooter(.25, .6
    //),() -> m_shooter.runShooter(0, 0), m_shooter));

    // shootPID = new JoystickButton(left, 1); //6 = right bumper
    // shootPID.whileHeld(
    //   new SequentialCommandGroup(
    //     new StartEndCommand(()->m_feeder.runFeeder(-.3), ()-> m_feeder.runFeeder(0), m_feeder).withTimeout(.3),
    //   new ParallelCommandGroup(
    //     new StartEndCommand(()->m_shooter.runTop(), ()->m_shooter.runShooter(0,0)),
    //     new StartEndCommand(()->m_shooter.runBottom(), ()->m_shooter.runShooter(0,0))
    //   ))
    // );

    shootPID = new JoystickButton(left, 1); //6 = right bumper
    shootPID.whileHeld(
      new SequentialCommandGroup(
        new StartEndCommand(()->m_feeder.runFeeder(-.3), ()-> m_feeder.runFeeder(0), m_feeder).withTimeout(.3),
      new ParallelCommandGroup(
        new StartEndCommand(()->m_shooter.runTop(), ()->m_shooter.runShooter(0,0)),
        new StartEndCommand(()->m_shooter.runBottom(), ()->m_shooter.runShooter(0,0)
        )
      )
    )
    );

    //Align Shooter
    new JoystickButton(left, 2)
      .whileHeld(new TurnToTarget(m_drive));
      
    runFeeder = new JoystickButton(right, 2); 
    runFeeder.whileHeld(new StartEndCommand(() -> m_feeder.runFeeder(.8),() -> m_feeder.runFeeder(0), m_feeder));

    //Reverse Feeder
    new JoystickButton(right, 5)
      .whileHeld(new StartEndCommand(() -> m_feeder.runFeeder(-1),() -> m_feeder.runFeeder(0), m_feeder));

    runIntake = new JoystickButton(right, 1); // 
    runIntake.whileHeld(new ParallelCommandGroup(new StartEndCommand(() -> m_intake.run(1),() -> m_intake.run(0), m_intake), new StartEndCommand(()-> m_lime.stream.setDouble(2), ()->m_lime.stream.setDouble(0), m_lime)));

    climbUp = new JoystickButton(right, 7); // DPad Up
    climbUp.whileHeld(new ParallelCommandGroup(new StartEndCommand(() -> m_climb.climbUp(-.6), () -> m_climb.climbUp(0), m_climb), new StartEndCommand(()->m_lime.ledMode.setNumber(2), ()-> m_lime.ledMode.setNumber(0), m_lime)));
    
    climbDown = new JoystickButton(right, 8); // DPad Down
    climbDown.whileHeld(new StartEndCommand(() -> m_climb.climbDown(.6),() -> m_climb.climbDown(0), m_climb));

    //turnToTarget = new JoystickButton(GP, 5); // left bumper
    //turnToTarget.whileHeld(new TurnToTarget(m_drive));

    toggleArm = new JoystickButton(left, 3); //left DPad
    toggleArm.whenPressed(new InstantCommand(() -> m_climb.togglePistons(), m_climb));

    toggleIntake = new JoystickButton(right, 4); // A Button
    toggleIntake.whenPressed(new InstantCommand(()->m_intake.toggleSolenoid(), m_intake));

    setIntake = new JoystickButton(GP, 9); // left thumb
    setIntake.whileHeld(new StartEndCommand(() -> m_intake.setIntake(true), () -> m_intake.setIntake(false), m_intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command m_auto = paths.get(trajNames[0]);
    Trajectory m_traj = pathsTrajs.get(trajNames[0]);

    m_drive.resetOdometry(m_traj.getInitialPose());

    // Push the trajectory to Field2d.
    m_drive.m_field.getObject("traj").setTrajectory(m_traj);

    // AUTONAV
    return m_auto.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }

  public void initPaths(){
    for(int i = 0; i < trajNames.length; i++){
      Trajectory t = genTraj(trajNames[i]);
      pathsTrajs.put(trajNames[i], t);
      paths.put(trajNames[i], genCommand(t));
    }  
  }

  public Trajectory genTraj(String path){
    String trajectoryJSON = path;
    Trajectory trajectory = new Trajectory();
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      }
    return trajectory;
  }

  public RamseteCommand genCommand(Trajectory trajectory){
    return new RamseteCommand(
      trajectory,
        m_drive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts,
                                  Constants.kvVoltSecondsPerMeter,
                                  Constants.kaVoltSecondsSquaredPerMeter),
        m_drive.m_kinematics,
        m_drive::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drive::tankDriveVolts,
        m_drive
    );
  }
}
