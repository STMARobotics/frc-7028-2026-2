package frc.robot.commands;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Command to intake fuel from the floor by deploying the intake and running the roller.
 */
public class IntakeCommand extends Command {

  private final IntakeSubsytem intakeSubsytem;
  private final LEDSubsystem ledSubsystem;
  private final LEDPattern patternOne = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlack, Color.kOrange)
      .scrollAtRelativeSpeed(Percent.per(Second).of(200));
  private final LEDPattern patternTwo = patternOne.reversed();

  private boolean hasDeployed = false;

  /**
   * Constructor for IntakeCommand
   * 
   * @param intakeSubsytem the intake subsystem
   * @param intakeLEDSubsystem the intake LED subsystem
   */
  public IntakeCommand(IntakeSubsytem intakeSubsytem, LEDSubsystem ledSubsystem) {
    this.intakeSubsytem = intakeSubsytem;
    this.ledSubsystem = ledSubsystem;

    addRequirements(intakeSubsytem, ledSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsytem.runIntake();
    intakeSubsytem.deploy();
    hasDeployed = false;
  }

  @Override
  public void execute() {
    if (hasDeployed || intakeSubsytem.isDeployed()) {
      hasDeployed = true;
      intakeSubsytem.stopDeploy();
    }
    ledSubsystem.runPatternOnHalvesAsCommand(patternOne, patternTwo);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsytem.stop();
    ledSubsystem.off();
  }

}
