package frc.robot.commands;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.LEDSubsystemContainer.IntakeLEDSubsystem;

/**
 * Command to intake fuel from the floor. This has a prerequisite of the intake being deployed.
 */
public class IntakeCommand extends Command {

  private final IntakeSubsytem intakeSubsytem;
  private final IntakeLEDSubsystem intakeLEDSubsystem;

  private final LEDPattern patternLeft = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlack, Color.kOrange)
      .scrollAtRelativeSpeed(Percent.per(Second).of(200));
  private final LEDPattern patternRight = patternLeft.reversed();

  public IntakeCommand(IntakeSubsytem intakeSubsytem, IntakeLEDSubsystem intakeLEDSubsystem) {
    this.intakeSubsytem = intakeSubsytem;
    this.intakeLEDSubsystem = intakeLEDSubsystem;

    addRequirements(intakeSubsytem, intakeLEDSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsytem.runIntake();
  }

  @Override
  public void execute() {
    intakeLEDSubsystem.runPatternOnIntakeHighLeft(patternLeft);
    intakeLEDSubsystem.runPatternOnIntakeHighRight(patternRight);
    intakeLEDSubsystem.runPatternOnIntakeLowLeft(patternLeft);
    intakeLEDSubsystem.runPatternOnIntakeLowRight(patternRight);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsytem.stopIntaking();

    intakeLEDSubsystem.off();
  }

}
