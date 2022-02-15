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

  ShuffleboardTab tab = Shuffleboard.getTab("Feeder System");

  private NetworkTableEntry FeederVRPM =     
  tab.add("Feeder Velocity (RPM)", this.feederMotor.getEncoder().getVelocity())
  .getEntry();
  private NetworkTableEntry FeederVP =     
  tab.add("Feeder Velocity (%)", this.speed)
  .getEntry();
  private NetworkTableEntry FeederPos =     
  tab.add("Feeder Position", this.feederMotor.getEncoder().getPosition())
  .getEntry();

  /** Creates a new Feeder. */
  public Feeder() {
    this.feederMotor = new CANSparkMax(Constants.FEEDER_MOTOR, MotorType.kBrushless);

    resetMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shuffleInit();
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
    this.FeederVRPM.setDouble(this.feederMotor.getEncoder().getVelocity());
    this.FeederVP.setDouble(this.speed);
    this.FeederPos.setDouble(this.feederMotor.getEncoder().getPosition());
  }
}