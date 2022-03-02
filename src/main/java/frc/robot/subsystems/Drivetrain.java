// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private CANSparkMax frontL, frontR, backL, backR;
  private MotorControllerGroup left, right;
  private DifferentialDrive drive;
  private double resetR, resetL;

  AHRS navx  = new AHRS(Port.kMXP);

  private DifferentialDriveOdometry m_odometry =
  new DifferentialDriveOdometry(navx.getRotation2d());

  public DifferentialDriveKinematics m_kinematics =
  new DifferentialDriveKinematics(Constants.TRACK_WIDTH_METERS);

  //FIELD
  public final Field2d m_field = new Field2d();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    frontL = new CANSparkMax(Constants.FRONTL, MotorType.kBrushless);
    backL = new CANSparkMax(Constants.BACKL, MotorType.kBrushless);
    frontR = new CANSparkMax(Constants.FRONTR, MotorType.kBrushless);
    backR = new CANSparkMax(Constants.BACKR, MotorType.kBrushless);

    left = new MotorControllerGroup(frontL, backL);
    right = new MotorControllerGroup(frontR, backR);

    drive = new DifferentialDrive(left, right);
    setIdleMode(IdleMode.kBrake);
    
    //SET CONVERSION FACTORS
    frontL.getEncoder().setPositionConversionFactor(Units.inchesToMeters(Constants.CIRCUMFERENCE) / Constants.GEAR_RATIO);
    frontR.getEncoder().setPositionConversionFactor(Units.inchesToMeters(Constants.CIRCUMFERENCE) / Constants.GEAR_RATIO);
    frontL.getEncoder().setVelocityConversionFactor(Units.inchesToMeters(Constants.CIRCUMFERENCE) / (60.0 * Constants.GEAR_RATIO));
    frontR.getEncoder().setVelocityConversionFactor(Units.inchesToMeters(Constants.CIRCUMFERENCE) / (60.0 * Constants.GEAR_RATIO));
    resetR = resetL = 0;

    resetEncoders();

    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Heading", navx.getRotation2d().getRadians());

    SmartDashboard.putNumber("Velocity FL", getLeftVelocity());
    SmartDashboard.putNumber("Position FL", getLeftEncoderPos());

    SmartDashboard.putNumber("Velocity FR", getRightVelocity());
    SmartDashboard.putNumber("Position FR", getRightEncoderPos());

    m_odometry.update(
      navx.getRotation2d(), getLeftEncoderPos(), getRightEncoderPos()
    );

    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    //System.out.println("Leftspeed: " + leftSpeed + "\t" + "Rightspeed: " + rightSpeed);
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    left.setVoltage(-leftVolts);
    right.setVoltage(-rightVolts);
    drive.feed();
  }

  public void arcadeDrive(double speed, double rot){
    drive.arcadeDrive(speed, rot);
  }

  private void setIdleMode(IdleMode mode) {
    frontL.setIdleMode(mode);
    frontR.setIdleMode(mode);
    backL.setIdleMode(mode);
    backR.setIdleMode(mode);
  }

  public void resetEncoders(){
    resetL = frontL.getEncoder().getPosition();
    resetR = -frontR.getEncoder().getPosition();
  }

  public double getRightEncoderPos(){
    return -frontR.getEncoder().getPosition() - resetR;
  }
  public double getLeftEncoderPos(){
    return frontL.getEncoder().getPosition() - resetL;
  }

  public double getLeftVelocity(){
    return frontL.getEncoder().getVelocity();
  }

  public double getRightVelocity(){
    return -frontR.getEncoder().getVelocity();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public double getHeading(){
    return navx.getAngle();
  }
  
  public void resetGyro(){
    navx.reset();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, navx.getRotation2d());
  }
}
