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
public class ToggleSolenoid extends InstantCommand {

  private DoubleSolenoid dSolenoid;
  private Solenoid solenoid;
  private boolean isDouble;

  public ToggleSolenoid(DoubleSolenoid dSolenoid) {
    super();
    this.dSolenoid = dSolenoid;
    isDouble = true;
  }

  public ToggleSolenoid(Solenoid solenoid) {
    super();
    this.solenoid = solenoid;
    isDouble = false;
  }

  // Called once when the command executes

  public void initialize() {
    if (!isDouble)
      solenoid.set(!solenoid.get());
    else
      dSolenoid.set((dSolenoid.get() == Value.kForward) ? Value.kReverse : Value.kForward);
  }
}
