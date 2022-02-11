// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  public CANSparkMax frontL, frontR, backL, backR;
  public MotorControllerGroup left, right;
  public DifferentialDrive drive;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    frontL = new CANSparkMax(Constants.FRONTL, MotorType.kBrushless);
    frontR = new CANSparkMax(Constants.FRONTR, MotorType.kBrushless);
    backL = new CANSparkMax(Constants.BACKL, MotorType.kBrushless);
    backR = new CANSparkMax(Constants.BACKR, MotorType.kBrushless);

    left = new MotorControllerGroup(frontL, backL);
    right = new MotorControllerGroup(frontR, backR);

    frontL.restoreFactoryDefaults();
    frontR.restoreFactoryDefaults();
    backL.restoreFactoryDefaults();
    backR.restoreFactoryDefaults();

    left.setInverted(true);

    drive = new DifferentialDrive(left, right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("P", frontL.getPIDController().getP());
    SmartDashboard.putNumber("I", frontL.getPIDController().getD());
    SmartDashboard.putNumber("D", frontL.getPIDController().getI());

    SmartDashboard.putNumber("Velocity FL", frontL.getEncoder().getVelocity());
    SmartDashboard.putNumber("Position FL", frontL.getEncoder().getPosition());
    SmartDashboard.putNumber("Temp FL", frontL.getMotorTemperature());

    SmartDashboard.putNumber("Velocity FR", frontR.getEncoder().getVelocity());
    SmartDashboard.putNumber("Position FR", frontR.getEncoder().getPosition());
    SmartDashboard.putNumber("Temp FR", frontR.getMotorTemperature());

    SmartDashboard.putNumber("Velocity BL", backL.getEncoder().getVelocity());
    SmartDashboard.putNumber("Position BL", backL.getEncoder().getPosition());
    SmartDashboard.putNumber("Temp BL", backL.getMotorTemperature());

    SmartDashboard.putNumber("Velocity BR", backR.getEncoder().getVelocity());
    SmartDashboard.putNumber("Position BR", backR.getEncoder().getPosition());
    SmartDashboard.putNumber("Temp BR", backR.getMotorTemperature());    
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    //The numbers come in from the Y-axis of the controller as - 
    drive.tankDrive(-leftSpeed, -rightSpeed);
  }
}