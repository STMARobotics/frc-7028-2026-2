package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsytem;

/**
 * Command to retract the intake and turn it off once retracted. This command will run until interrupted.
 */
public class RetractIntakeCommand extends Command {

  private final IntakeSubsytem intakeSubsystem;
  private boolean hasRetracted = false;

  public RetractIntakeCommand(IntakeSubsytem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.retract();
    hasRetracted = false;
  }

  @Override
  public void execute() {
    if (hasRetracted || intakeSubsystem.isRetracted()) {
      // Turn the intake off once it's retracted
      hasRetracted = true;
      intakeSubsystem.stopDeploy();
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
  }

}
