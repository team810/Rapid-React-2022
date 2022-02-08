// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Drivetrain extends SubsystemBase {

  public CANSparkMax frontL, frontR, backL, backR;

  public DifferentialDrive diffDrive;

  private double resetR = 0;
  private double resetL = 0;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    frontL = new CANSparkMax(Constants.FRONTL, MotorType.kBrushless);
    backL = new CANSparkMax(Constants.BACKL, MotorType.kBrushless);
    frontR = new CANSparkMax(Constants.FRONTR, MotorType.kBrushless);
    backL = new CANSparkMax(Constants.BACKL, MotorType.kBrushless);

    backL.restoreFactoryDefaults();
    backR.restoreFactoryDefaults();
    frontR.restoreFactoryDefaults();
    frontL.restoreFactoryDefaults();



    //If you remove this, it doesnt work. We dont know why
    frontL.follow(ExternalFollower.kFollowerDisabled, 0);
    backL.follow(frontL);

    diffDrive = new DifferentialDrive(frontL, frontR);

    diffDrive.setSafetyEnabled(false);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    if(Math.abs(leftSpeed) < .1){
      leftSpeed = 0;
    }
    if(Math.abs(rightSpeed) < .1){
      rightSpeed = 0;
    }
    //System.out.println(leftSpeed + " : " + rightSpeed);
    diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void resetEncoders(){
    resetL = frontL.getEncoder().getPosition();
    resetR = frontR.getEncoder().getPosition();
  }

  public double getRightEncoderPos(){
    return frontR.getEncoder().getPosition() - resetR;
  }
  public double getLeftEncoderPos(){
    return frontL.getEncoder().getPosition() - resetL;
  }


}
