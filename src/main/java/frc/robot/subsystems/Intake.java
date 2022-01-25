// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  public CANSparkMax intake;
  public DoubleSolenoid intakeSolLeft, intakeSolRight;

  /** Creates a new Intake. */
  public Intake() {
    intake = new CANSparkMax(Constants.INTAKE, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void toggleIntake(boolean value, double speed){
    intakeSolLeft.set(Value.kForward);
    intakeSolRight.set(Value.kForward);
  }

}
