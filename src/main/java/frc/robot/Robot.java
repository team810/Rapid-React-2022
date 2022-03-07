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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

//  private Intake m_intake;

  private RobotContainer m_robotContainer;
  public Compressor c;
  Drivetrain m_drive;
  String directory = "pathplanner/generatedJSON/";
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_drive = m_robotContainer.m_drive;

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
    m_robotContainer.paths.put(m_robotContainer.trajNames[0], genCommand(t));

    //Simple Blue Auto 2
    t = genTraj(directory + "Simple_Blue_Auto_2.wpilib.json");
    m_robotContainer.pathsTrajs.put(m_robotContainer.trajNames[1], t);
    m_robotContainer.paths.put(m_robotContainer.trajNames[1], genCommand(t));

    //Simple Blue Auto 3
    t = genTraj(directory + "Simple_Blue_Auto_3.wpilib.json");
    m_robotContainer.pathsTrajs.put(m_robotContainer.trajNames[2], t);
    m_robotContainer.paths.put(m_robotContainer.trajNames[2], genCommand(t));

    //Three Ball Blue
    
    
    //Four Ball Blue


    //Simple Red Auto 1

    //Simple Red Auto 2

    //Simple Red Auto 3

    //Three Red Blue

    //Four Red Blue
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    //i.setIntake(false);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
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
  public void autonomousPeriodic() {}

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
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
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
  }
}
