// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private CANSparkMax feederMotor;
  private double speed;

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
  }

  private void shuffleInit() {
    SmartDashboard.putNumber("Feeder Velocity (RPM)", this.feederMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Feeder Velocity (%)", this.speed);

    SmartDashboard.putNumber("Feeder Position", this.feederMotor.getEncoder().getPosition());
  }
}