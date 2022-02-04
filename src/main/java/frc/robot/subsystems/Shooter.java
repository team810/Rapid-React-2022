// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase 
{
  public CANSparkMax motor1, motor2;
  /** Creates a new Shooter. */
  public Shooter() 
  {
    motor1 = new CANSparkMax(Constants.SHOOTER_MOTOR, MotorType.kBrushless);
    motor2 = new CANSparkMax(Constants.SHOOTER_MOTOR, MotorType.kBrushless);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
  public void set(double speed)
  {
    motor1.set(speed);
    motor2.set(speed);
  }
}
