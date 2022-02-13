// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  public CANSparkMax intake;
  public Solenoid intakeSol;
  private boolean bool;

  /** Creates a new Intake. */
  public Intake() {
    intake = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
    intakeSol = new Solenoid(PneumaticsModuleType.REVPH, 9);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Velcocity (RPM)", intake.getEncoder().getVelocity());
    SmartDashboard.putNumber("Intake Position", intake.getEncoder().getPosition());

    SmartDashboard.putBoolean("Solenoid on?", this.bool);
  }

  public void runIntake(double speed, boolean bool) {
    intake.set(speed);
    intakeSol.set(bool);
    this.bool = bool;
  }
}