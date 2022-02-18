// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private CANSparkMax climberMotor;
  private double speed;
  private DoubleSolenoid leftHook, rightHook; 

  NetworkTableEntry speedRPM, speedPercent, climberPosition;

  ShuffleboardTab tab = Shuffleboard.getTab("Climber System");

  public Climber() {
    this.climberMotor = new CANSparkMax(Constants.CLIMBER_MOTOR, MotorType.kBrushless);
    this.leftHook = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.HOOKL_1, Constants.HOOKL_2);
    this.rightHook = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.HOOKR_1, Constants.HOOKR_2);

    motorReset();

    shuffleInit();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shuffleUpdate();
  }

  public void runClimber(double speed) {
    this.climberMotor.set(speed);
    this.speed = speed;
    while (this.climberMotor.getEncoder().getPosition() < Constants.CLIMBER_REVS) {}
    this.climberMotor.set(0);
  }

  public void toggleLeftHook(Value value)
  {
    leftHook.set(value);
  }

  public void toggleRightHook(Value value)
  {
    rightHook.set(value);
  }

  private void motorReset() {
    this.climberMotor.restoreFactoryDefaults();

    this.climberMotor.setIdleMode(IdleMode.kBrake);
  }

  public void shuffleInit() {
    speedRPM = tab.add("Climber Velocity (RPM)", this.climberMotor.getEncoder().getVelocity()).getEntry();
    speedPercent = tab.add("Climber Velocity (%)", this.speed).getEntry();
    climberPosition = tab.add("Climber Position", this.climberMotor.getEncoder().getPosition()).getEntry();
  }

  public void shuffleUpdate() {
    speedRPM.setDouble(this.climberMotor.getEncoder().getVelocity());
    speedPercent.setDouble(this.speed);
    climberPosition.setDouble(this.climberMotor.getEncoder().getPosition());

  }
}
