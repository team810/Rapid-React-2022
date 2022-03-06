// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

<<<<<<< HEAD
import edu.wpi.first.wpilibj.motorcontrol.Spark;
=======
import java.awt.Color;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
<<<<<<< HEAD
  private Spark feederMotor;

  /** Creates a new Feeder. */
  public Feeder() {
    this.feederMotor = new Spark(Constants.FEEDER_MOTOR);
=======

  Spark feederMotor;
  ColorSensorV3 colorSensorV3;

  /** Creates a new Indexer. */
  public Feeder() {

    feederMotor = new Spark(Constants.FEEDER);
    colorSensorV3 = new ColorSensorV3(Port.kOnboard);
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
<<<<<<< HEAD
  }

  public void runFeeder(double speed) {
    this.feederMotor.set(speed);
  }
}
=======
    SmartDashboard.putNumber("Red", colorSensorV3.getRed());
    SmartDashboard.putNumber("Green", colorSensorV3.getGreen());
    SmartDashboard.putNumber("Blue", colorSensorV3.getBlue());
    SmartDashboard.putNumber("Proximity", colorSensorV3.getProximity());
  }

  public void runFeeder(double speed){
    feederMotor.set(speed);
  }

}
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f
