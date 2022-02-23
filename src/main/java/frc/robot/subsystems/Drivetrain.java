// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
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
  private double leftSpeed, rightSpeed;
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

    frontL.follow(ExternalFollower.kFollowerDisabled, 0);
    frontR.follow(ExternalFollower.kFollowerDisabled, 0);
    //this.left.setInverted(true);

    //this.drive = new DifferentialDrive(left, right);

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
    //
    shuffleInit();

    m_odometry.update(
      navx.getRotation2d(), getLeftEncoderPos(), getRightEncoderPos()
    );

    m_field.setRobotPose(m_odometry.getPoseMeters());
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



  public void arcadeDrive(double speed, double rot){
    //ystem.out.println(rot);
    drive.arcadeDrive(speed, rot);
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
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, navx.getRotation2d());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontR.setVoltage(rightVolts);
    frontL.setVoltage(leftVolts);
    //System.out.println(leftVolts + " : " + rightVolts);
    drive.feed();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(frontL.getEncoder().getVelocity(), frontR.getEncoder().getVelocity());
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public double getHeading(){
    return Math.IEEEremainder(navx.getAngle(), 360) * -1;
  }

  private void shuffleInit() {
    SmartDashboard.putNumber("Heading", getHeading());

    SmartDashboard.putNumber("Velocity FL", frontL.getEncoder().getVelocity());
    SmartDashboard.putNumber("Position FL", frontL.getEncoder().getPosition());
    SmartDashboard.putNumber("Temp FL", frontL.getMotorTemperature());

    SmartDashboard.putNumber("Velocity FR", frontR.getEncoder().getVelocity());
    SmartDashboard.putNumber("Position FR", frontR.getEncoder().getPosition());
    SmartDashboard.putNumber("Temp FR", frontR.getMotorTemperature());
  }
}