package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  public CANSparkMax intake;
  public Solenoid intakeSolLeft, intakeSolRight;

  /** Creates a new Intake. */
  public Intake() {
    intake = new CANSparkMax(Constants.INTAKE, MotorType.kBrushless);
    intakeSolLeft = new Solenoid(PneumaticsModuleType.REVPH, 9);
    intakeSolRight = new Solenoid(PneumaticsModuleType.REVPH, 8);

  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  public void setIntake(boolean value, double speed){
    intakeSolLeft.set(value);
    intakeSolRight.set(value);
    intake.setIdleMode(IdleMode.kBrake);
    intake.set(speed);
  }

}
