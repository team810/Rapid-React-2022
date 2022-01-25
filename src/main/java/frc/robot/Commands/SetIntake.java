// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetIntake extends ParallelCommandGroup {


  Intake i;
  /** Creates a new ToggleIntake. */
  public SetIntake(Intake i, boolean value) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Drive forward the specified distance
      new SetSolenoid(i.intakeSolLeft, value),

      // Release the hatch
      new SetSolenoid(i.intakeSolRight, value));

      // Drive backward the specified distance
     

  }
}
