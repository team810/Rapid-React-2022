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
  private CANSparkMax frontL, frontR, backL, backR;
  private MotorControllerGroup left, right;
  private DifferentialDrive drive;

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
    this.drive.tankDrive(-leftSpeed, -rightSpeed);
  }

  private void resetMotors() {
    this.frontL.restoreFactoryDefaults();
    this.frontR.restoreFactoryDefaults();
    this.backL.restoreFactoryDefaults();
    this.backR.restoreFactoryDefaults();
  }

  private void shuffleInit() {
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
}