// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private CANSparkMax frontL, frontR, backL, backR;
  private MotorControllerGroup left, right;
  private DifferentialDrive drive;
  private double leftSpeed, rightSpeed; 

  ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain System");

  private NetworkTableEntry VelocityLP =     
    tab.add("Drivetrain Velocity Left (%)", this.leftSpeed)
  .getEntry();
  private NetworkTableEntry VelocityRP =     
  tab.add("Drivetrain Velocity Right (%)", this.rightSpeed)
  .getEntry();

  private NetworkTableEntry VelocityFL =     
  tab.add("Velocity FL", frontL.getEncoder().getVelocity())
  .getEntry();
  private NetworkTableEntry PositionFL =     
  tab.add("Position FL", frontL.getEncoder().getPosition())
  .getEntry();
  private NetworkTableEntry TempFL =     
  tab.add("Temp FL", frontL.getMotorTemperature())
  .getEntry();

  private NetworkTableEntry VelocityFR =     
  tab.add("Velocity FR", frontR.getEncoder().getVelocity())
  .getEntry();
  private NetworkTableEntry PositionFR =     
  tab.add("Position FR", frontR.getEncoder().getPosition())
  .getEntry();
  private NetworkTableEntry TempFR =     
  tab.add("Temp FR", frontR.getMotorTemperature())
  .getEntry();

  private NetworkTableEntry VelocityBL =     
  tab.add("Velocity FR", backL.getEncoder().getVelocity())
  .getEntry();
  private NetworkTableEntry PositionBL =     
  tab.add("Position BL", backL.getEncoder().getPosition())
  .getEntry();
  private NetworkTableEntry TempBL =     
  tab.add("Temp BL", backL.getMotorTemperature())
  .getEntry();

  private NetworkTableEntry VelocityBR =     
  tab.add("Velocity BR", backR.getEncoder().getVelocity())
  .getEntry();
  private NetworkTableEntry PositionBR =     
  tab.add("Position BR", backR.getEncoder().getPosition())
  .getEntry();
  private NetworkTableEntry TempBR =     
  tab.add("Temp BR", backR.getMotorTemperature())
  .getEntry();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    this.frontL = new CANSparkMax(Constants.FRONTL, MotorType.kBrushless);
    this.frontR = new CANSparkMax(Constants.FRONTR, MotorType.kBrushless);
    this.backL = new CANSparkMax(Constants.BACKL, MotorType.kBrushless);
    this.backR = new CANSparkMax(Constants.BACKR, MotorType.kBrushless);

    resetMotors();

    this.left = new MotorControllerGroup(frontL, backL);
    this.right = new MotorControllerGroup(frontR, backR);

    this.left.setInverted(true);

    this.drive = new DifferentialDrive(left, right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shuffleInit();
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    // The numbers come in from the Y-axis of the controller as -, reversed them to
    // positive before passing
    this.drive.tankDrive(-leftSpeed, rightSpeed);

    this.leftSpeed = -leftSpeed;
    this.rightSpeed = rightSpeed;
  }

  private void resetMotors() {
    this.frontL.restoreFactoryDefaults();
    this.frontR.restoreFactoryDefaults();
    this.backL.restoreFactoryDefaults();
    this.backR.restoreFactoryDefaults();

    this.frontL.setIdleMode(IdleMode.kBrake);
    this.frontR.setIdleMode(IdleMode.kBrake);
    this.backL.setIdleMode(IdleMode.kBrake);
    this.backR.setIdleMode(IdleMode.kBrake);
  }

  private void shuffleInit() {
    VelocityLP.setDouble(this.leftSpeed);
    VelocityRP.setDouble(this.rightSpeed);

    VelocityFL.setDouble(frontL.getEncoder().getVelocity());
    PositionFL.setDouble(frontL.getEncoder().getPosition());
    TempFL.setDouble(frontL.getMotorTemperature());
    
    VelocityFR.setDouble(frontR.getEncoder().getVelocity());
    PositionFR.setDouble(frontR.getEncoder().getPosition());
    TempFR.setDouble(frontR.getMotorTemperature());
    
    VelocityBL.setDouble(backL.getEncoder().getVelocity());
    PositionBL.setDouble(backL.getEncoder().getPosition());
    TempBL.setDouble(backL.getMotorTemperature());
    
    VelocityBR.setDouble(backR.getEncoder().getVelocity());
    PositionBR.setDouble(backR.getEncoder().getPosition());
    TempBR.setDouble(backR.getMotorTemperature());
  }
}