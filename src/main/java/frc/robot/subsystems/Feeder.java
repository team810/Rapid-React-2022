// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {

  CANSparkMax feederMotor;

  /** Creates a new Indexer. */
  public Feeder() {

    feederMotor = new CANSparkMax(Constants.FEEDER, MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runFeeder(){
    feederMotor.set(.7);
  }
}
