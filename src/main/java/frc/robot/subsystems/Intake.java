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

public class Intake extends SubsystemBase 
{
  private double speed;
  private boolean bool1, bool2;
  public CANSparkMax intake;
  public Solenoid intakeSolLeft, intakeSolRight;

  /** Creates a new Intake. */
  public Intake() 
  {
    intake = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
    intakeSolLeft = new Solenoid(PneumaticsModuleType.REVPH, 9);
    intakeSolRight = new Solenoid(PneumaticsModuleType.REVPH, 8);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Speed", this.speed);
    SmartDashboard.putBoolean("Left Solenoid on?", this.bool1);
    SmartDashboard.putBoolean("Left Solenoid on?", this.bool2);

  }
  public void set(double speed)
  {
    intake.set(speed);
    this.speed = speed;
  }
  public void setLeftSolenoid(boolean bool)
  {
    intakeSolLeft.set(bool);
    intakeSolRight.set(bool);

    this.bool1 = bool;
    this.bool2 = bool;
  }
}
