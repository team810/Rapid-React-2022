// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private CANSparkMax feederMotor;
  // Ultrasonic colorSensor;
  private double speed;

  ShuffleboardTab tab = Shuffleboard.getTab("Feeder System");

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
    tab.add("Feeder Velocity (RPM)", this.feederMotor.getEncoder().getVelocity());
    tab.add("Feeder Velocity (%)", this.speed);
    tab.add("Feeder Position", this.feederMotor.getEncoder().getPosition());
  }
}