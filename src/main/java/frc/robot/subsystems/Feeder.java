// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private CANSparkMax feederMotor;
  // Ultrasonic colorSensor;
  private double speed;

  NetworkTableEntry speedRPM, speedPercent, feederPosition;

  ShuffleboardTab tab = Shuffleboard.getTab("Feeder System");

  /** Creates a new Feeder. */
  public Feeder() {
    this.feederMotor = new CANSparkMax(Constants.FEEDER_MOTOR, MotorType.kBrushless);

    resetMotors();

    shuffleInit();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shuffleUpdate();
  }

  public void runFeeder(double speed) {
    this.feederMotor.set(speed);
    
    this.speed = speed;
  }

  private void resetMotors() {
    this.feederMotor.restoreFactoryDefaults();

    this.feederMotor.setIdleMode(IdleMode.kCoast);
    
    // Ultrasonic.setAutomaticMode(true);
  }

  private void shuffleInit() {
    speedRPM = tab.add("Feeder Velocity (RPM)", this.feederMotor.getEncoder().getVelocity()).getEntry();
    speedPercent = tab.add("Feeder Velocity (%)", this.speed).getEntry();
    feederPosition = tab.add("Feeder Position", this.feederMotor.getEncoder().getPosition()).getEntry();
  }

  private void shuffleUpdate() {
    speedRPM.setDouble(this.feederMotor.getEncoder().getVelocity());
    speedPercent.setDouble(this.speed);
    feederPosition.setDouble(this.feederMotor.getEncoder().getPosition());
  }
}