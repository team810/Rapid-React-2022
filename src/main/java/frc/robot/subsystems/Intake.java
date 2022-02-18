// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor;
  private Solenoid intakeSol;
  private boolean bool;
  private double speed;

  NetworkTableEntry speedRPM, speedPercent, intakePosition, solenoidState;

  ShuffleboardTab tab = Shuffleboard.getTab("Intake System");

  /** Creates a new Intake. */
  public Intake() {
    this.intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
    this.intakeSol = new Solenoid(PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID);
    this.intakeSol.set(bool);

    resetMotors();

    shuffleInit();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shuffleUpdate();
  }

  public void runIntake(double speed) {
    this.intakeMotor.set(speed);

    this.speed = speed;
  }

  private void resetMotors() {
    this.intakeMotor.restoreFactoryDefaults();

    this.intakeMotor.setIdleMode(IdleMode.kCoast);
  }

  private void shuffleInit() {
    speedRPM = tab.add("Intake Velcocity (RPM)", this.intakeMotor.getEncoder().getVelocity()).getEntry();
    speedPercent = tab.add("Intake Velcocity (%)", this.speed).getEntry();
    intakePosition = tab.add("Intake Position", this.intakeMotor.getEncoder().getPosition()).getEntry();
    solenoidState = tab.add("Solenoid on?", this.bool).getEntry(); // this is odd because idk whether to do addBoolean or add
  }

  private void shuffleUpdate() {
    speedRPM.setDouble(this.intakeMotor.getEncoder().getVelocity());
    speedPercent.setDouble(this.speed);
    intakePosition.setDouble(this.intakeMotor.getEncoder().getPosition());
    solenoidState.setBoolean(this.bool);
  }
}