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

public class Shooter extends SubsystemBase {
  public CANSparkMax top, bottom;

  /** Creates a new Shooter. */
  public Shooter() {
    top = new CANSparkMax(Constants.SHOOTER_TOP, MotorType.kBrushless);
    bottom = new CANSparkMax(Constants.SHOOTER_BOTTOM, MotorType.kBrushless);

    /*
     * this decreaces the time between shots
     * by leaving the motors at a higher speed when not in use
     * 
     */
    top.setIdleMode(IdleMode.kCoast);
    bottom.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Velocity Top(RPM)", top.getEncoder().getVelocity());
    SmartDashboard.putNumber("Velocity Bottom(RPM)", top.getEncoder().getVelocity());
  }

  public void run(double topS, double bottomS) {
    top.set(-topS);
    bottom.set(bottomS);
  }
}