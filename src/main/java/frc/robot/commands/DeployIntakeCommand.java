package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsytem;

/**
 * Command to deploy the intake. This command will run until the intake is fully deployed.
 */
public class DeployIntakeCommand extends Command {

  private final IntakeSubsytem intakeSubsystem;

  public DeployIntakeCommand(IntakeSubsytem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.deploy();
  }

  @Override
  public boolean isFinished() {
    return intakeSubsystem.isDeployed();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopDeploy();
  }

}
