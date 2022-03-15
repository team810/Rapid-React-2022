// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootSequenceTest extends SequentialCommandGroup {
  /** Creates a new ShootSequenceTest. */
  public ShootSequenceTest(Drivetrain m_drive, Feeder m_feeder, Shooter m_shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        //Reverse shooter and feeder for .3 seconds, and then full feeder forward
        new StartEndCommand(()->m_feeder.runFeeder(-.3), ()-> m_feeder.runFeeder(0), m_feeder).withTimeout(.3),
        //Turn to target
        new TurnToTarget(m_drive)
      ),
      new StartEndCommand(()->m_shooter.run(), ()->m_shooter.runShooter(0,0), m_shooter).withInterrupt(m_shooter.atSetpoint()),
      new ParallelDeadlineGroup(
        new StartEndCommand(()->m_shooter.run(), ()->m_shooter.runShooter(0,0), m_shooter).withInterrupt(m_shooter.outsideSetpoint()),
        new StartEndCommand(()->m_feeder.runFeeder(1), ()-> m_feeder.runFeeder(0), m_feeder)
      ),
      new ParallelCommandGroup(
        //Reverse shooter and feeder for .3 seconds, and then full feeder forward
        new StartEndCommand(()->m_feeder.runFeeder(-.3), ()-> m_feeder.runFeeder(0), m_feeder).withTimeout(.3),
        //Turn to target
        new TurnToTarget(m_drive)
      ),
      new StartEndCommand(()->m_shooter.run(), ()->m_shooter.runShooter(0,0), m_shooter).withInterrupt(m_shooter.atSetpoint()),
      new ParallelDeadlineGroup(
        new StartEndCommand(()->m_shooter.run(), ()->m_shooter.runShooter(0,0), m_shooter).withInterrupt(m_shooter.outsideSetpoint()),
        new StartEndCommand(()->m_feeder.runFeeder(1), ()-> m_feeder.runFeeder(0), m_feeder)
      )

    );
  }
}
