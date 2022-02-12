// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  public CANSparkMax intake;
  public Solenoid intakeSolLeft, intakeSolRight;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /** Creates a new Intake. */
  public Intake() {
    intake = new CANSparkMax(Constants.INTAKE, MotorType.kBrushless);
    intakeSolLeft = new Solenoid(PneumaticsModuleType.REVPH, 9);
    intakeSolRight = new Solenoid(PneumaticsModuleType.REVPH, 8);

  }

  @Override
  public void periodic() {

    ejectBall();


  }

  public void setIntake(boolean value, double speed) {
    intakeSolLeft.set(value);
    intakeSolRight.set(value);
    intake.setIdleMode(IdleMode.kBrake);
    intake.set(speed);
  }

  public boolean getCorrectBall() {

    Color detectedColor = m_colorSensor.getColor();

    if (DriverStation.getAlliance() == DriverStation.getAlliance().Red) {
      if (detectedColor.red > .5) {
        return true;
      } else {
        return false;
      }
    } else if (DriverStation.getAlliance() == DriverStation.getAlliance().Blue) {
      if (detectedColor.blue > .5) {
        return true;
      } else {
        return false;
      }
    }

    return false;

  }

  public void ejectBall(){
    if(!getCorrectBall()){
      setIntake(false, -.7);
    }
  }

}
