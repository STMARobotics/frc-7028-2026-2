package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsytem;

/**
 * Command to deploy the intake and turn it off once deployed. This command will run until interrupted.
 */
public class DeployIntakeCommand extends Command {

  private final IntakeSubsytem intakeSubsystem;
  private boolean hasDeployed = false;

  public DeployIntakeCommand(IntakeSubsytem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.deploy();
    hasDeployed = false;
  }

  @Override
  public void execute() {
    if (hasDeployed || intakeSubsystem.isDeployed()) {
      // Turn the intake off once it's deployed
      hasDeployed = true;
      intakeSubsystem.stopDeploy();
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopDeploy();
  }

}
