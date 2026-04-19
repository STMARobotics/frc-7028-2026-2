package frc.robot.commands;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * A command to run everything backwards to eject or unjam fuel.
 */
public class EjectCommand extends Command {

  private final IntakeSubsytem intakeSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final LEDSubsystem ledSubsystem;

  private boolean hasDeployed = false;

  /**
   * Constructor for EjectCommand
   * 
   * @param intakeSubsystem intake subsystem
   * @param indexerSubsystem indexer subsystem
   * @param feederSubsystem feeder subsystem
   * @param shooterSubsystem shooter subsystem
   * @param ledSubsystem LED subsystem
   */
  public EjectCommand(
      IntakeSubsytem intakeSubsystem,
      IndexerSubsystem indexerSubsystem,
      FeederSubsystem feederSubsystem,
      ShooterSubsystem shooterSubsystem,
      LEDSubsystem ledSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.ledSubsystem = ledSubsystem;

    addRequirements(intakeSubsystem, indexerSubsystem, feederSubsystem, shooterSubsystem, ledSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.eject();
    indexerSubsystem.eject();
    feederSubsystem.eject();
    shooterSubsystem.eject();
    hasDeployed = false;
  }

  @Override
  public void execute() {
    if (hasDeployed || intakeSubsystem.isDeployed()) {
      // Turn the intake off once it's deployed
      hasDeployed = true;
      intakeSubsystem.stopDeploy();
    }
    ledSubsystem.runPattern(LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of(200)));
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
    indexerSubsystem.stop();
    feederSubsystem.stop();
    shooterSubsystem.stop();
    ledSubsystem.off();
  }

}
