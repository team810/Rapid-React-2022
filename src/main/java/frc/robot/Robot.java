// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Commands.ShootSequence;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

//  private Intake m_intake;

  private RobotContainer m_robotContainer;
  public Compressor c;
  Drivetrain m_drive;
  Feeder m_feeder;
  Intake m_intake;
  Shooter m_shooter;

  String directory = "pathplanner/generatedJSON/";
  

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
<<<<<<< HEAD
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
=======
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_drive = m_robotContainer.m_drive;
    m_feeder = m_robotContainer.m_feeder;
    m_intake = m_robotContainer.m_intake;
    m_shooter = m_robotContainer.m_shooter;

    //GEN COMMANDS
    // for(int i = 0; i < m_robotContainer.trajNames.length; i++){
    //   String traj = "pathplanner/generatedJSON/" + m_robotContainer.trajNames[i] + ".wpilib.json";
    //   Trajectory t = genTraj(traj);
    //   m_robotContainer.pathsTrajs.put(m_robotContainer.trajNames[i], t);
    //   m_robotContainer.paths.put(m_robotContainer.trajNames[i], genCommand(t));
    // }  

    //Simple Blue Auto 1
    Trajectory t = genTraj(directory + "Simple_Blue_Auto_1.wpilib.json");
    m_robotContainer.pathsTrajs.put(m_robotContainer.trajNames[0], t);
    SequentialCommandGroup command = new SequentialCommandGroup(
      new ParallelDeadlineGroup(
          genCommand(t),
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder)
        ),
        new ShootSequence(m_drive, m_feeder, m_shooter).withTimeout(5)
    ).andThen(()->m_drive.tankDriveVolts(0, 0));

    m_robotContainer.paths.put(m_robotContainer.trajNames[0], command);

    //Simple Blue Auto 2
    t = genTraj(directory + "Simple_Blue_Auto_2.wpilib.json");
    m_robotContainer.pathsTrajs.put(m_robotContainer.trajNames[1], t);
    command = new SequentialCommandGroup(
      new ParallelDeadlineGroup(
          genCommand(t),
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder)
        ),
        new ShootSequence(m_drive, m_feeder, m_shooter).withTimeout(5)
    ).andThen(()->m_drive.tankDriveVolts(0, 0));
    m_robotContainer.paths.put(m_robotContainer.trajNames[1], command);

    //Simple Blue Auto 3
    t = genTraj(directory + "Simple_Blue_Auto_3.wpilib.json");
    m_robotContainer.pathsTrajs.put(m_robotContainer.trajNames[2], t);
    command = new SequentialCommandGroup(
      new ParallelDeadlineGroup(
          genCommand(t),
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder)
        ),
        new ShootSequence(m_drive, m_feeder, m_shooter).withTimeout(5)
    ).andThen(()->m_drive.tankDriveVolts(0, 0));
    m_robotContainer.paths.put(m_robotContainer.trajNames[2], command);

    //Three Ball Blue
    
      //First do simple blue auto 2, then three ball blue p1
      t = genTraj(directory + "Simple_Blue_Auto_2.wpilib.json");
      m_robotContainer.pathsTrajs.put(m_robotContainer.trajNames[3], t);
      RamseteCommand com1 = genCommand(t);
      t = genTraj(directory + "Three_Ball_Blue_P1.wpilib.json");
      RamseteCommand com2 = genCommand(t);
      command = new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          com1,
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder)
        ),
        new ShootSequence(m_drive, m_feeder, m_shooter).withTimeout(5),
        new ParallelDeadlineGroup(
          com2,
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder)
        ),
        new ShootSequence(m_drive, m_feeder, m_shooter).withTimeout(5)
      ).andThen(()->m_drive.tankDriveVolts(0, 0));
      m_robotContainer.paths.put(m_robotContainer.trajNames[3], command);

    //TODO: Four Ball Blue




    //Simple Red Auto 1
    t = genTraj(directory + "Simple_Red_Auto_1.wpilib.json");
    m_robotContainer.pathsTrajs.put(m_robotContainer.trajNames[4], t);
    command = new SequentialCommandGroup(
      new ParallelDeadlineGroup(
          genCommand(t),
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder)
        ),
        new ShootSequence(m_drive, m_feeder, m_shooter).withTimeout(5)
    ).andThen(()->m_drive.tankDriveVolts(0, 0));

    m_robotContainer.paths.put(m_robotContainer.trajNames[4], command);

    //Simple Red Auto 2
    t = genTraj(directory + "Simple_Red_Auto_2.wpilib.json");
    m_robotContainer.pathsTrajs.put(m_robotContainer.trajNames[5], t);
    command = new SequentialCommandGroup(
      new ParallelDeadlineGroup(
          genCommand(t),
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder)
        ),
        new ShootSequence(m_drive, m_feeder, m_shooter).withTimeout(5)
    ).andThen(()->m_drive.tankDriveVolts(0, 0));
    m_robotContainer.paths.put(m_robotContainer.trajNames[5], command);

    //Simple Red Auto 3
    t = genTraj(directory + "Simple_Red_Auto_3.wpilib.json");
    m_robotContainer.pathsTrajs.put(m_robotContainer.trajNames[6], t);
    command = new SequentialCommandGroup(
      new ParallelDeadlineGroup(
          genCommand(t),
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder)
        ),
        new ShootSequence(m_drive, m_feeder, m_shooter).withTimeout(5)
    ).andThen(()->m_drive.tankDriveVolts(0, 0));
    m_robotContainer.paths.put(m_robotContainer.trajNames[6], command);

    //Three Ball Blue
    
      //First do simple blue auto 2, then three ball blue p1
      t = genTraj(directory + "Simple_Red_Auto_2.wpilib.json");
      m_robotContainer.pathsTrajs.put(m_robotContainer.trajNames[7], t);
      com1 = genCommand(t);
      t = genTraj(directory + "Three_Ball_Red_P1.wpilib.json");
      com2 = genCommand(t);
      command = new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          com1,
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder)
        ),
        new ShootSequence(m_drive, m_feeder, m_shooter).withTimeout(5),
        new ParallelDeadlineGroup(
          com2,
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder)
        ),
        new ShootSequence(m_drive, m_feeder, m_shooter).withTimeout(5)
      ).andThen(()->m_drive.tankDriveVolts(0, 0));
      m_robotContainer.paths.put(m_robotContainer.trajNames[7], command);

    //TODO: Four Ball Red
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
<<<<<<< HEAD
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
=======
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
<<<<<<< HEAD

  }

  @Override
  public void disabledPeriodic() {

  }
=======
    //i.setIntake(false);
  }

  @Override
  public void disabledPeriodic() {}
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
<<<<<<< HEAD
  public void autonomousPeriodic() {

  }
=======
  public void autonomousPeriodic() {}
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
<<<<<<< HEAD
  public void teleopPeriodic() {

  }
=======
  public void teleopPeriodic() {}
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
<<<<<<< HEAD
  public void testPeriodic() {

=======
  public void testPeriodic() {}

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
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f
  }
}
