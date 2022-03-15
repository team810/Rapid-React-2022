// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootSequenceCopy extends ParallelCommandGroup {
  /** Creates a new ShootSequence. */
  public ShootSequenceCopy(Drivetrain m_drive, Feeder m_feeder, Shooter m_shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //Reverse shooter and feeder for .3 seconds, and then full feeder forward
        new StartEndCommand(()->m_feeder.runFeeder(-.3), ()-> m_feeder.runFeeder(0), m_feeder).withTimeout(.3)
          .andThen(new StartEndCommand(() -> m_feeder.runFeeder(1),() -> m_feeder.runFeeder(0), m_feeder)),

      //Turn to target
      new TurnToTarget(m_drive),

      //Run top and bottom shooters using limelight given RPM values
      new StartEndCommand(()->m_shooter.runTop(), ()->m_shooter.runShooter(0,0)),
      new StartEndCommand(()->m_shooter.runBottom(), ()->m_shooter.runShooter(0,0))
    );

  }
}
