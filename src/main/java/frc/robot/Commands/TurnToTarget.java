package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TurnToTarget extends PIDCommand {
  /** 
   * Command that aims the robot to the central hub by using the limelight
   */
  public TurnToTarget(Drivetrain m_drive) {
    super(
        // PID Values
        new PIDController(.05, 0.001, 0.0035),
        // Measurement is the tx that the limelight returns
        () -> Constants.tx.getDouble(0),
        // Setpoint is always 0, as the delta x must be 0 for a line up
        () -> 0,
        // Output command
        output -> {
          m_drive.arcadeDrive(0, output);
        });

    addRequirements(m_drive);
    // Set tolerance to +/- .2, as that is "good enough"
    getController().setTolerance(.2);
  }

  // Returns true when the robot is at setpoint
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
