// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private CANSparkMax frontL, frontR, backL, backR;
  private MotorControllerGroup left, right;
  private DifferentialDrive drive;
  private double leftSpeed, rightSpeed;

  AHRS ahrs;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    this.frontL = new CANSparkMax(Constants.FRONTL, MotorType.kBrushless);
    this.frontR = new CANSparkMax(Constants.FRONTR, MotorType.kBrushless);
    this.backL = new CANSparkMax(Constants.BACKL, MotorType.kBrushless);
    this.backR = new CANSparkMax(Constants.BACKR, MotorType.kBrushless);

    this.drive = new DifferentialDrive(frontL, frontL);
    resetMotors();

    //this.left = new MotorControllerGroup(frontL, backL);
    //this.right = new MotorControllerGroup(frontR, backR);

    backL.follow(frontL);
    backR.follow(frontR);

    ahrs = new AHRS(Port.kMXP);

    //this.left.setInverted(true);

    //this.drive = new DifferentialDrive(left, right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //shuffleInit();
    

  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    // The numbers come in from the Y-axis of the controller as -, reversed them to
    // positive before passing
    //this.drive.tankDrive(-leftSpeed, -rightSpeed);
    if(Math.abs(leftSpeed) < .02){
      leftSpeed = 0;
    }
    if(Math.abs(rightSpeed) < .02){
      rightSpeed = 0;
    }

    frontL.set(leftSpeed);
    frontR.set(rightSpeed);

    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
  }

  public void slowTankDrive(double leftSpeed, double rightSpeed) {
    // The numbers come in from the Y-axis of the controller as -, reversed them to
    // positive before passing
    //this.drive.tankDrive(-leftSpeed, -rightSpeed);
    if(Math.abs(leftSpeed) < .02){
      leftSpeed = 0;
    }
    if(Math.abs(rightSpeed) < .02){
      rightSpeed = 0;
    }

    frontL.set(-leftSpeed / 2);
    frontR.set(rightSpeed / 2);

    this.leftSpeed = -leftSpeed / 2;
    this.rightSpeed = rightSpeed / 2;
  }



  public void arcadeDrive(double speed, double rot){
    System.out.println(rot);
    drive.arcadeDrive(speed, rot);
    // if(rot > 0){
    //   System.out.print();
    //   //too far left, turn right
    //   frontL.set(-rot);
    //   frontR.set(rot);
    // }else{
    //   //too far right, turn left
    //   frontL.set(rot);
    //   frontR.set(-rot);
    // }
  }

  private void resetMotors() {
    // this.frontL.restoreFactoryDefaults();
    // this.frontR.restoreFactoryDefaults();
    // this.backL.restoreFactoryDefaults();
    // this.backR.restoreFactoryDefaults();

    this.frontL.setIdleMode(IdleMode.kBrake);
    this.frontR.setIdleMode(IdleMode.kBrake);
    this.backL.setIdleMode(IdleMode.kBrake);
    this.backR.setIdleMode(IdleMode.kBrake);
  }

  private void shuffleInit() {
    SmartDashboard.putNumber("P", frontL.getPIDController().getP());
    SmartDashboard.putNumber("I", frontL.getPIDController().getD());
    SmartDashboard.putNumber("D", frontL.getPIDController().getI());

    SmartDashboard.putNumber("Drivetrain Velcoity Left (%)", this.leftSpeed);
    SmartDashboard.putNumber("Drivetrain Velocity Right (%)", this.rightSpeed);

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