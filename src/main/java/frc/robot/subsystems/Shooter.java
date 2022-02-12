// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  public static CANSparkMax shooter, active, hood;

  /** Creates a new Shooter. */
  public Shooter() {

    shooter = new CANSparkMax(Constants.SHOOTER, MotorType.kBrushless);
    active = new CANSparkMax(Constants.ACTIVE, MotorType.kBrushless);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runShooter(){
     shooter.set(1);
     active.set(.7);
  }

  public CANSparkMax getShooter(){
    return shooter;
  }

  public CANSparkMax getActive(){
    return active;
  }
}
