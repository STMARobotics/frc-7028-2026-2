package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsytem;

/**
 * Command to retract the intake. This command will hold the intake indefinitely until interrupted.
 */
public class RetractIntakeCommand extends Command {

  private final IntakeSubsytem intakeSubsystem;

  public RetractIntakeCommand(IntakeSubsytem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void execute() {
    if (intakeSubsystem.isRetracted()) {
      // Once retracted, hold it in place
      intakeSubsystem.holdRetracted();
    } else {
      // Not retracted, so retract it
      intakeSubsystem.retract();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopDeploy();
  }

}
