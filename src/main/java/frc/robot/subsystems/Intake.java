// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private Spark intakeMotor;
  private DoubleSolenoid intakeSol;

  /** Creates a new Intake. */
  public Intake() {
    this.intakeMotor = new Spark(Constants.INTAKE_MOTOR);
    this.intakeSol = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID_1, Constants.INTAKE_SOLENOID_2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake(double speed, Value value, int num) {
    this.intakeSol.set(value);
    this.intakeMotor.set(speed);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(num);
  }
}