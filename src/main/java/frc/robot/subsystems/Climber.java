// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public CANSparkMax climberMotor;
  private double speed;

  private DoubleSolenoid sol;

  public Climber() {
    this.climberMotor = new CANSparkMax(Constants.CLIMB, MotorType.kBrushless);

    motorReset();
    climberMotor.setIdleMode(IdleMode.kCoast);

    sol = new DoubleSolenoid(PneumaticsModuleType.REVPH, 14, 15);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shuffleInit();

    //System.out.println(climberMotor.getEncoder().getPosition());



  }



  public void climbUp(double speed){
    this.climberMotor.set((speed));
  }

  public void climbDown(double speed){
    this.climberMotor.set((speed));
  }

  private void motorReset() {
    this.climberMotor.restoreFactoryDefaults();
    this.climberMotor.setIdleMode(IdleMode.kBrake);
  }

  public void shuffleInit() {



    SmartDashboard.putNumber("Climber Position", -climberMotor.getEncoder().getPosition());

    SmartDashboard.putBoolean("Climber At Top", climberMotor.getEncoder().getPosition() < -85 ? true : false);    
    SmartDashboard.putBoolean("Hook Position", sol.get() == Value.kForward ? false : true);
  }

  public void togglePistons() {
    if (sol.get() == Value.kForward) {
      sol.set(Value.kReverse);
    } else {
      sol.set(Value.kForward);
    }
  }

  public void setPistons(boolean value) {
    if (value) {
      sol.set(Value.kReverse);
    } else {
      sol.set(Value.kForward);
    }
  }

  

}