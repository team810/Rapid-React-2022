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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.ShootSequence;
import frc.robot.Commands.ShootSequenceTest;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

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
  Feeder m_feeder;
  Intake m_intake;
  Shooter m_shooter;

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
    m_feeder = m_robotContainer.m_feeder;
    m_intake = m_robotContainer.m_intake;
    m_shooter = m_robotContainer.m_shooter;



    //Simple Blue Auto 1, shoot low goal and then shoot high
    Trajectory t = genTraj(directory + "Simple_Blue_Auto_1.wpilib.json");
    m_robotContainer.pathsTrajs.put("Simple Blue Auto 1 Low/High", t);
    SequentialCommandGroup command = new SequentialCommandGroup(
      //Shoot low goal for 2 seconds
      new ParallelDeadlineGroup(
        new StartEndCommand(()->m_shooter.runShooter(.3, .3), ()->m_shooter.runShooter(0, 0), m_shooter).withTimeout(2),
        new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder)
      ),
      //Run trajectory, and then shoot high
      new ParallelDeadlineGroup(
        //Run traj and wait for 2 seconds
        new SequentialCommandGroup(
          genCommand(t),
          new WaitCommand(2)
        ),
        //Run intake and feeder
        new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
        new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder),
        new StartEndCommand(()->m_shooter.run(), ()->m_shooter.runShooter(0, 0), m_shooter)
      )
    ).andThen(()->m_drive.tankDriveVolts(0, 0));

    m_robotContainer.paths.put("Simple Blue Auto 1 Low/High", command);
    
    // OR Shoot 2 high
    m_robotContainer.pathsTrajs.put("Simple Blue Auto 1 High/High", t);
    command = new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          genCommand(t),
          new WaitCommand(2)
        ),
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder),
          new StartEndCommand(()->m_shooter.run(), ()->m_shooter.runShooter(0, 0), m_shooter)
      )
    ).andThen(()->m_drive.tankDriveVolts(0, 0));

    m_robotContainer.paths.put("Simple Blue Auto 1 High/High", command); 



    //Simple Blue Auto 2 Shoot Low then High
    t = genTraj(directory + "Simple_Blue_Auto_2.wpilib.json");
    m_robotContainer.pathsTrajs.put("Simple Blue Auto 2 Low/High", t);
    command = new SequentialCommandGroup(
      //Shoot low goal for 2 seconds
      new ParallelDeadlineGroup(
        new StartEndCommand(()->m_shooter.runShooter(.2, .2), ()->m_shooter.runShooter(0, 0), m_shooter).withTimeout(2),
        new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder)
      ),
      new WaitCommand(0),
      //Run trajectory, and then shoot high
      new ParallelDeadlineGroup(
        //Run traj and wait for 2 seconds
        new SequentialCommandGroup(
          genCommand(t),
          new WaitCommand(2)
        ),
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder),
          new StartEndCommand(()->m_shooter.run(), ()->m_shooter.runShooter(0, 0), m_shooter)
        )
    ).andThen(()->m_drive.tankDriveVolts(0, 0));

    m_robotContainer.paths.put("Simple Blue Auto 2 Low/High", command);

    //OR shoot 2 high
    t = genTraj(directory + "Simple_Blue_Auto_2.wpilib.json");
    m_robotContainer.pathsTrajs.put("Simple Blue Auto 2 High/High", t);
    command = new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          genCommand(t),
          new WaitCommand(0.2)
        ),
        new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
        new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0))
      )
    ).andThen(
      new SequentialCommandGroup(
        new StartEndCommand(()->m_feeder.runFeeder(-1), ()->m_feeder.runFeeder(0)).withTimeout(.4),
        new ShootSequence(m_drive, m_feeder, m_shooter)
    )).andThen(()->m_drive.tankDriveVolts(0, 0));
    m_robotContainer.paths.put("Simple Blue Auto 2 High/High", command);

    //OR do 3 auto by getting adjacent ball <-- PICK UP FROM HERE
    
    t = genTraj(directory + "Simple_Blue_Auto_2.wpilib.json");
    m_robotContainer.pathsTrajs.put("Three Blue Ball Adjacent", t);
    RamseteCommand com1 = genCommand(t);
    t = genTraj(directory + "Three_Ball_Blue_P1.wpilib.json");
    RamseteCommand com2 = genCommand(t);
    command = new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          com1,
          new WaitCommand(1)
        ),
        new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
        new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder)
      ).andThen(new ShootSequence(m_drive, m_feeder, m_shooter)),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          com2,
          new WaitCommand(1)
        ),
        new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
        new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder)
      ).andThen(new ShootSequence(m_drive, m_feeder, m_shooter))
    ).andThen(()->m_drive.tankDriveVolts(0, 0));
    m_robotContainer.paths.put("Three Blue Ball Adjacent", command);

    //4 BALL AUTO ---------------

    //Shoot 1 low 1 high
    t = genTraj(directory + "Simple_Blue_Auto_2.wpilib.json");
    m_robotContainer.pathsTrajs.put("4 Ball Auto", t);
    Command command1 = new SequentialCommandGroup(
      //Shoot low goal for 2 seconds
      new ParallelDeadlineGroup(
        new StartEndCommand(()->m_shooter.runShooter(.2, .2), ()->m_shooter.runShooter(0, 0), m_shooter).withTimeout(2),
        new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder)
      ),
      new WaitCommand(0),
      //Run trajectory, and then shoot high
      new ParallelDeadlineGroup(
        //Run traj and wait for 2 seconds
        new SequentialCommandGroup(
          genCommand(t),
          new WaitCommand(2)
        ),
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder),
          new StartEndCommand(()->m_shooter.run(), ()->m_shooter.runShooter(0, 0), m_shooter)
        )
    );
    
    t = genTraj(directory + "4_Ball_Auto.wpilib.json");
    // Follow trajectory to player station and back
    Command command2 = new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          genCommand(t),
          new WaitCommand(.1)
        ),
        new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
        new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0))
      ));

    command = new SequentialCommandGroup(
      command1,
      command2
    ).andThen(
      new SequentialCommandGroup(
        new StartEndCommand(()->m_feeder.runFeeder(-1), ()->m_feeder.runFeeder(0)).withTimeout(.4),
        new ShootSequence(m_drive, m_feeder, m_shooter))
      )
      .andThen(()->m_drive.tankDriveVolts(0, 0));

    /*
    //Simple Blue Auto 3 shoot low and then shoot high
    t = genTraj(directory + "Simple_Blue_Auto_3.wpilib.json");
    m_robotContainer.pathsTrajs.put("Simple Blue Auto 3 Low/High", t);
    command = new SequentialCommandGroup(
      //Shoot low goal for 2 seconds
      new ParallelDeadlineGroup(
        new StartEndCommand(()->m_shooter.runShooter(.3, .3), ()->m_shooter.runShooter(0, 0), m_shooter).withTimeout(2),
        new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder)
      ),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          genCommand(t),
          new WaitCommand(1)
        ),
        new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
        new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder),
        new StartEndCommand(()->m_shooter.run(), ()->m_shooter.runShooter(0, 0), m_shooter)
      )
    ).andThen(()->m_drive.tankDriveVolts(0, 0));
    m_robotContainer.paths.put("Simple Blue Auto 3 Low/High", command);

    //OR shoot 2 high
    m_robotContainer.pathsTrajs.put("Simple Blue Auto 3 High/High", t);
    command = new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          genCommand(t),
          new WaitCommand(1)
        ),
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder),
          new StartEndCommand(()->m_shooter.run(), ()->m_shooter.runShooter(0, 0), m_shooter)
        )
    ).andThen(()->m_drive.tankDriveVolts(0, 0));
    m_robotContainer.paths.put("Simple Blue Auto 3 High/High", command);





    //Simple Red Auto 1 shoot low and then high
    t = genTraj(directory + "Simple_Red_Auto_1.wpilib.json");
    m_robotContainer.pathsTrajs.put("Simple Red Auto 1 Low/High", t);
    command = new SequentialCommandGroup(
      //Shoot low goal for 2 seconds
      new ParallelDeadlineGroup(
        new StartEndCommand(()->m_shooter.runShooter(.3, .3), ()->m_shooter.runShooter(0, 0), m_shooter).withTimeout(2),
        new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder)
      ),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          genCommand(t),
          new WaitCommand(1)
        ),
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder),
          new StartEndCommand(()->m_shooter.run(), ()->m_shooter.runShooter(0, 0), m_shooter)
        )
    ).andThen(()->m_drive.tankDriveVolts(0, 0));

    m_robotContainer.paths.put("Simple Red Auto 1 Low/High", command);

    //OR 2 high
    t = genTraj(directory + "Simple_Red_Auto_1.wpilib.json");
    m_robotContainer.pathsTrajs.put("Simple Red Auto 1 High/High", t);
    command = new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          genCommand(t),
          new WaitCommand(1)
        ),
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder),
          new StartEndCommand(()->m_shooter.run(), ()->m_shooter.runShooter(0, 0), m_shooter)
        )
    ).andThen(()->m_drive.tankDriveVolts(0, 0));

    m_robotContainer.paths.put("Simple Red Auto 1 High/High", command);



    //Simple Red Auto 2
    t = genTraj(directory + "Simple_Red_Auto_2.wpilib.json");
    m_robotContainer.pathsTrajs.put("Simple Red Auto 2 Low/High", t);
    command = new SequentialCommandGroup(
      //Shoot low goal for 2 seconds
      new ParallelDeadlineGroup(
        new StartEndCommand(()->m_shooter.runShooter(.3, .3), ()->m_shooter.runShooter(0, 0), m_shooter).withTimeout(2),
        new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder)
      ),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          genCommand(t),
          new WaitCommand(1)
        ),
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder),
          new StartEndCommand(()->m_shooter.run(), ()->m_shooter.runShooter(0, 0), m_shooter)
        )
    ).andThen(()->m_drive.tankDriveVolts(0, 0));
    m_robotContainer.paths.put("Simple Red Auto 2 Low/High", command);

    //OR 2 high
    t = genTraj(directory + "Simple_Red_Auto_2.wpilib.json");
    m_robotContainer.pathsTrajs.put("Simple Red Auto 2 High/High", t);
    command = new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          genCommand(t),
          new WaitCommand(1)
        ),
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder),
          new StartEndCommand(()->m_shooter.run(), ()->m_shooter.runShooter(0, 0), m_shooter)
        )
    ).andThen(()->m_drive.tankDriveVolts(0, 0));
    m_robotContainer.paths.put("Simple Red Auto 2 High/High", command);



    //Simple Red Auto 3 shoot low then high
    t = genTraj(directory + "Simple_Red_Auto_3.wpilib.json");
    m_robotContainer.pathsTrajs.put("Simple Red Auto 3 Low/High", t);
    command = new SequentialCommandGroup(
      //Shoot low goal for 2 seconds
      new ParallelDeadlineGroup(
        new StartEndCommand(()->m_shooter.runShooter(.3, .3), ()->m_shooter.runShooter(0, 0), m_shooter).withTimeout(2),
        new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder)
      ),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          genCommand(t),
          new WaitCommand(1)
        ),
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder),
          new StartEndCommand(()->m_shooter.run(), ()->m_shooter.runShooter(0, 0), m_shooter)
        )
    ).andThen(()->m_drive.tankDriveVolts(0, 0));
    m_robotContainer.paths.put("Simple Red Auto 3 Low/High", command);

    //OR 2 high
    t = genTraj(directory + "Simple_Red_Auto_3.wpilib.json");
    m_robotContainer.pathsTrajs.put("Simple Red Auto 3 High/High", t);
    command = new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          genCommand(t),
          new WaitCommand(1)
        ),
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder),
          new StartEndCommand(()->m_shooter.run(), ()->m_shooter.runShooter(0, 0), m_shooter)
        )
    ).andThen(()->m_drive.tankDriveVolts(0, 0));
    m_robotContainer.paths.put("Simple Red Auto 3 High/High", command);



    //Three Ball Red
    
      //First do simple blue auto 2, then three ball blue p1
      t = genTraj(directory + "Simple_Red_Auto_2.wpilib.json");
      m_robotContainer.pathsTrajs.put("Three Ball Red Adjacent", t);
      com1 = genCommand(t);
      t = genTraj(directory + "Three_Ball_Red_P1.wpilib.json");
      com2 = genCommand(t);
      command = new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new SequentialCommandGroup(
          com1,
          new WaitCommand(1)
        ),
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder),
          new StartEndCommand(()->m_shooter.run(), ()->m_shooter.runShooter(0, 0), m_shooter)
        ),
        new ParallelDeadlineGroup(
          new SequentialCommandGroup(
          com2,
          new WaitCommand(1)
        ),
          new StartEndCommand(()-> m_intake.setIntake(true), ()->m_intake.run(0)),
          new StartEndCommand(()->m_feeder.runFeeder(1), ()->m_feeder.runFeeder(0), m_feeder),
          new StartEndCommand(()->m_shooter.run(), ()->m_shooter.runShooter(0, 0), m_shooter)
        )
      ).andThen(()->m_drive.tankDriveVolts(0, 0));
      m_robotContainer.paths.put("Three Ball Red Adjacent", command);*/
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
