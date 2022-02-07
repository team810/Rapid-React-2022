// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.util.function.DoubleSupplier;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class Drivetrain extends SubsystemBase 
// {
//   public DoubleSupplier leftSpeed, rightSpeed;
//   public CANSparkMax frontL, frontR, backL, backR;
//   public MotorControllerGroup left, right;
//   public DifferentialDrive drive;

//   /** Creates a new Drivetrain. */
//   public Drivetrain() 
//   {
//     frontL = new CANSparkMax(Constants.FRONTL, MotorType.kBrushless);
//     frontR = new CANSparkMax(Constants.FRONTR, MotorType.kBrushless);
//     backL = new CANSparkMax(Constants.BACKL, MotorType.kBrushless);
//     backR = new CANSparkMax(Constants.BACKR, MotorType.kBrushless);

//     left = new MotorControllerGroup(frontL, backL);
//     right = new MotorControllerGroup(frontR, backR);

//     frontL.restoreFactoryDefaults();
//     frontR.restoreFactoryDefaults();
//     backL.restoreFactoryDefaults();
//     backR.restoreFactoryDefaults();

//     left.setInverted(true);

//     drive = new DifferentialDrive(left, right);
//   }

//   @Override
//   public void periodic() 
//   {
//     // This method will be called once per scheduler run
//     SmartDashboard.putNumber("Velocity", frontL.getEncoder().getVelocity());
//     SmartDashboard.putNumber("Position FL", frontL.getEncoder().getPosition());
//     SmartDashboard.putNumber("Position FR", frontL.getEncoder().getPosition());
//     SmartDashboard.putNumber("Position BL", frontL.getEncoder().getPosition());
//     SmartDashboard.putNumber("Position BR", frontL.getEncoder().getPosition());
//   }
//   public void tankDrive(double leftSpeed, double rightSpeed)
//   {
//     drive.tankDrive(leftSpeed, rightSpeed);
//   }
// }