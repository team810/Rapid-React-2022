// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private Spark intakeMotor;
  private Solenoid intakeSol;
  private boolean bool;
  private double speed;

  ShuffleboardTab tab = Shuffleboard.getTab("Intake System");

  private NetworkTableEntry IntakeVP =     
  tab.add("Intake Velocity (%)", this.speed)
  .getEntry();
  private NetworkTableEntry IntakeSol = 
  tab.add("Solenoid on?", (this.bool))
  .getEntry();


  /** Creates a new Intake. */
  public Intake() {
    this.intakeMotor = new Spark(Constants.INTAKE_MOTOR);
    this.intakeSol = new Solenoid(PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID);
    this.intakeSol.set(bool);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shuffleUpdate();
  }

  public void toggleIntakeSol()
  {
    this.intakeSol.set(!IntakeSol.getBoolean(false));
    this.bool = IntakeSol.getBoolean(false);
  }

  public void runIntake(double speed) {
    this.intakeMotor.set(speed);

    this.speed = speed;
  }

  private void shuffleUpdate() {
    this.IntakeVP.setDouble(this.speed);
    this.IntakeSol.setBoolean(this.bool);
  }
}