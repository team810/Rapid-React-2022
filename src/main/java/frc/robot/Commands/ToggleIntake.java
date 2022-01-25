// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleIntake extends ParallelCommandGroup {

  Intake i;
  double speed;
  boolean value;
  /** Creates a new OpenIntake. */
  public ToggleIntake(Intake i, boolean value, double speed) {

    this.i = i;
    this.speed = speed;
    this.value = value;
    addRequirements(i);

    addCommands(
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    new SetIntake(i, value),
    new RunIntakeMotor(i, speed));

    // Release the hatch
   
  }
}
