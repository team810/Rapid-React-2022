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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor;
  private Solenoid intakeSol;
  private boolean bool;
  private double speed;

  ShuffleboardTab tab = Shuffleboard.getTab("Intake System");

  
  private NetworkTableEntry IntakeVRPM =     
  tab.add("Intake Velcocity (RPM)", this.intakeMotor.getEncoder().getVelocity())
  .getEntry();
  private NetworkTableEntry IntakeVP =     
  tab.add("Intake Velcocity (%)", this.speed)
  .getEntry();
  private NetworkTableEntry IntakePos =     
  tab.add("Intake Position", this.intakeMotor.getEncoder().getPosition())
  .getEntry();
  private NetworkTableEntry IntakeSol = 
  tab.add("Solenoid on?", (this.bool))
  .getEntry();


  /** Creates a new Intake. */
  public Intake() {
    this.intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
    this.intakeSol = new Solenoid(PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID);
    
    resetMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shuffleInit();
  }

  public void runIntake(double speed, boolean bool) {
    this.intakeMotor.set(speed);
    this.intakeSol.set(bool);

    this.bool = bool;
    this.speed = speed;
  }

  private void resetMotors() {
    this.intakeMotor.restoreFactoryDefaults();

    this.intakeMotor.setIdleMode(IdleMode.kCoast);
  }

  private void shuffleInit() {
    this.IntakeVRPM.setDouble(this.intakeMotor.getEncoder().getVelocity());
    this.IntakeVP.setDouble(this.speed);
    this.IntakePos.setDouble(this.intakeMotor.getEncoder().getPosition());
    this.IntakeSol.setBoolean(this.bool);
  }
}