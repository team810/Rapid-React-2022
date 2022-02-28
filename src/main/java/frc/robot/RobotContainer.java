// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  //AUTONOMOUS PATHS
  public HashMap<String, Trajectory> pathsTrajs = new HashMap<String, Trajectory>();
  public HashMap<String, Command> paths = new HashMap<String, Command>();
  public String[] trajNames = {"pathplanner/generatedJSON/Simple_Auto_1.wpilib.json"};

  //AUTONOMOUS PATH CHOOSER
  public SendableChooser<String> m_chooser = new SendableChooser<String>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //SUBSYTEM INITIAL TASKS
    m_drive.setDefaultCommand(
      new RunCommand(
        ()-> m_drive.tankDrive((left.getRawAxis(1)), (right.getRawAxis(1)))
        , m_drive
    )); 

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

  public void robotState(){
    double robot_velocity = (m_drive.getLeftVelocity() + m_drive.getRightVelocity()) / 2;
    double robot_heading = m_drive.getHeading(); 
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Shoot withOUT PID
    //new JoystickButton(left, 1);
    //.whileHeld(new StartEndCommand(() -> m_shooter.runShooter(.25, .6),() -> m_shooter.runShooter(0, 0), m_shooter));

    //Shoot with PID
    new JoystickButton(left, 1)
      .whileHeld(new ShootSequence(m_drive, m_feeder, m_shooter));

    //Align Shooter
    new JoystickButton(left, 2)
      .whileHeld(new TurnToTarget(m_drive));
    
    //Run Intake
    new JoystickButton(right, 1)
    .whileHeld(
      new ParallelCommandGroup(
        new StartEndCommand(() -> m_intake.run(1),() -> m_intake.run(0), m_intake),
        new StartEndCommand(()-> m_lime.stream.setDouble(2), ()->m_lime.stream.setDouble(0), m_lime)
      )
    );

    //Run Feeder
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
    new JoystickButton(right, 13)
      .whileHeld(
        new ParallelCommandGroup(
          new StartEndCommand(() -> m_climb.climbUp(-.6), () -> m_climb.climbUp(0), m_climb),
          new StartEndCommand(()->m_lime.ledMode.setNumber(2), ()-> m_lime.ledMode.setNumber(0), m_lime)
        )
      );
    
    //Climb DOWN
    new JoystickButton(right, 12)
      .whileHeld(new StartEndCommand(() -> m_climb.climbDown(.6),() -> m_climb.climbDown(0), m_climb));

    //Toggle climber arms
    new JoystickButton(right, 11)
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

    /*
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            m_drive.m_kinematics,
            7);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(m_drive.m_kinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, 1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand command = new RamseteCommand(
      exampleTrajectory,
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

    m_drive.resetOdometry(exampleTrajectory.getInitialPose());*/

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
