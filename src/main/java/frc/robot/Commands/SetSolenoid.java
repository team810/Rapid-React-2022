// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetSolenoid extends InstantCommand {

   /**
   * Add your docs here.
   */

  boolean isDouble, value;
  Solenoid solenoid;
  DoubleSolenoid dSolenoid;

  public SetSolenoid(Solenoid solenoid, boolean value) {
    super();
    isDouble = false;
    this.solenoid = solenoid;
    this.value = value;
  }

  public SetSolenoid(DoubleSolenoid solenoid, boolean value) {
    super();
    isDouble = true;
    this.dSolenoid = solenoid;
    this.value = value;
  }

  // Called once when the command executes

  public void initialize() {
    if (isDouble)
      dSolenoid.set((value) ? Value.kForward : Value.kReverse);
    else
      solenoid.set(value);
  }
}
