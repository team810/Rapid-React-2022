// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase 
{
  public DoubleSupplier leftSpeed, rightSpeed;
  public CANSparkMax frontL, frontR, backL, backR;
  public MotorControllerGroup left = new MotorControllerGroup(frontL, backL);
  public MotorControllerGroup right = new MotorControllerGroup(frontR, backR);
  public DifferentialDrive drive;
  public ShuffleboardTab M_DRIVETRAINTAB = Shuffleboard.getTab("Drivetrain"); 

  /** Creates a new Drivetrain. */
  public Drivetrain() 
  {
    frontL = new CANSparkMax(Constants.FRONTL, MotorType.kBrushless);
    frontR = new CANSparkMax(Constants.FRONTR, MotorType.kBrushless);
    backL = new CANSparkMax(Constants.BACKL, MotorType.kBrushless);
    backR = new CANSparkMax(Constants.BACKR, MotorType.kBrushless);

    frontL.restoreFactoryDefaults();
    frontR.restoreFactoryDefaults();
    backL.restoreFactoryDefaults();
    backR.restoreFactoryDefaults();

    left.setInverted(true);

    drive = new DifferentialDrive(left, right);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
  public void tankDrive(double leftSpeed, double rightSpeed)
  {
    drive.tankDrive(leftSpeed, rightSpeed);
  }
}