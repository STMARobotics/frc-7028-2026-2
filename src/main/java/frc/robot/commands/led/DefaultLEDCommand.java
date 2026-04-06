package frc.robot.commands.led;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj.LEDPattern.GradientType.kContinuous;
import static edu.wpi.first.wpilibj.LEDPattern.gradient;
import static edu.wpi.first.wpilibj.LEDPattern.solid;
import static edu.wpi.first.wpilibj.util.Color.kBlack;
import static edu.wpi.first.wpilibj.util.Color.kBlue;
import static edu.wpi.first.wpilibj.util.Color.kDarkRed;
import static edu.wpi.first.wpilibj.util.Color.kIndianRed;
import static edu.wpi.first.wpilibj.util.Color.kOrange;
import static frc.robot.subsystems.LEDSubsystem.candyCane;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

/**
 * The default command for controlling the LEDs
 */
public class DefaultLEDCommand extends Command {

  private static final Time CANDY_CANE_SPEED = Seconds.of(0.5);
  private static final LEDPattern dsDetachedPattern = candyCane(kDarkRed, kIndianRed, CANDY_CANE_SPEED);
  private static final LEDPattern disabledPattern = gradient(kContinuous, kBlue, kOrange)
      .scrollAtRelativeSpeed(Percent.per(Second).of(75));
  private static final LEDPattern testModePattern = candyCane(kOrange, kBlack, CANDY_CANE_SPEED);
  private static final LEDPattern enabledPatternOne = solid(Color.kBlue);
  private static final LEDPattern enabledPatternTwo = solid(Color.kOrange);

  private final LEDSubsystem ledSubsystem;

  /**
   * Creates a new DefaultLEDCommand
   * 
   * @param ledSubsystem LED subsystem
   */
  public DefaultLEDCommand(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;
    addRequirements(ledSubsystem);
  }

  @Override
  public void execute() {
    if (!DriverStation.isDSAttached()) {
      ledSubsystem.runPattern(dsDetachedPattern);
    } else if (RobotState.isDisabled()) {
      ledSubsystem.runPattern(disabledPattern);
    } else if (RobotState.isTest()) {
      ledSubsystem.runPattern(testModePattern);
    } else {
      if (Timer.getTimestamp() % 1 < 0.5) {
        ledSubsystem.runPatternOnHalves(enabledPatternOne, enabledPatternTwo);
      } else {
        ledSubsystem.runPatternOnHalves(enabledPatternTwo, enabledPatternOne);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    ledSubsystem.off();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
