// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {

  Spark feederMotor;
  ColorSensorV3 colorSensorV3;

  /** Creates a new Indexer. */
  public Feeder() {

    feederMotor = new Spark(Constants.FEEDER);
    colorSensorV3 = new ColorSensorV3(Port.kOnboard);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Red", colorSensorV3.getRed());
    SmartDashboard.putNumber("Green", colorSensorV3.getGreen());
    SmartDashboard.putNumber("Blue", colorSensorV3.getBlue());
    SmartDashboard.putNumber("Proximity", colorSensorV3.getProximity());
  }

  public void runFeeder(double speed) {
    feederMotor.set(speed);
  }
}
