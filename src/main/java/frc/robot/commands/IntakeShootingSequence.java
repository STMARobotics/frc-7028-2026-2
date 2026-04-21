package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.IntakeConstants.DEPLOY_SHOOTING_CURRENT;
import static frc.robot.Constants.ShootingConstants.JAM_DEBOUNCE_TIME;
import static frc.robot.Constants.ShootingConstants.JAM_THRESHOLD;
import static frc.robot.Constants.ShootingConstants.RETRACTED_THRESHOLD;
import static frc.robot.Constants.ShootingConstants.RETRACT_INTAKE_DELAY;
import static frc.robot.Constants.ShootingConstants.UNJAM_DURATION;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.IntakeSubsytem;

/**
 * Encapsulates the intake shooting sequence, since it's shared by multiple commands. The sequence handles retracting
 * the intake, and doing an unjam sequence.
 */
public class IntakeShootingSequence {
  private final IntakeSubsytem intakeSubsystem;

  private final Timer unJamTimer = new Timer();
  private final Timer retractDelayTimer = new Timer();
  private Debouncer jamDebouncer;
  private boolean hasRetracted = false;

  /**
   * Constructor for ShootingSequence.
   *
   * @param intakeSubsytem intake subsystem
   */
  public IntakeShootingSequence(IntakeSubsytem intakeSubsytem) {
    this.intakeSubsystem = intakeSubsytem;
  }

  /**
   * Resets the shooting sequence state with a tunable debounce time. Call this in command initialize().
   * 
   * @param debounceTime the time the debouncer should debounce jams
   */
  public void reset(Time debounceTime) {
    unJamTimer.stop();
    unJamTimer.reset();
    retractDelayTimer.stop();
    retractDelayTimer.reset();
    jamDebouncer = new Debouncer(debounceTime.in(Seconds));
    hasRetracted = false;
  }

  /**
   * Resets the shooting sequence state with the default debounce time. Call this in command initialize().
   */
  public void reset() {
    reset(JAM_DEBOUNCE_TIME);
  }

  /**
   * Runs one iteration of the intake shooting sequence with default parameters. This should be called repeatedly (e.g.
   * in command execute())
   */
  public void execute() {
    execute(RETRACT_INTAKE_DELAY, DEPLOY_SHOOTING_CURRENT, JAM_THRESHOLD, UNJAM_DURATION, RETRACTED_THRESHOLD);
  }

  /**
   * Runs one iteration of the intake shooting sequence with tunable unjam parameters. This should be called repeatedly
   * (e.g. in command execute())
   * 
   * @param retractDelay the time to wait before retracting the intake
   * @param retractCurrent the current to use when retracting the intake
   * @param jamThreshold the velocity below which we consider the intake to be jammed
   * @param unjamDuration the duration for which to unjamming
   * @param retractedThreshold the deploy position below which we consider the intake retracted and can stop the
   */
  public void execute(
      Time retractDelay,
      Current retractCurrent,
      AngularVelocity jamThreshold,
      Time unjamDuration,
      Angle retractedThreshold) {
    intakeSubsystem.runIntakeForShooting();
    retractDelayTimer.start();
    if (!retractDelayTimer.hasElapsed(retractDelay)) {
      // Don't retract for a fixed time at the start of the sequence
      return;
    }
    if (hasRetracted || intakeSubsystem.getDeployPosition().lte(retractedThreshold)) {
      intakeSubsystem.stopDeploy();
      hasRetracted = true;
    } else {
      boolean isJammed = intakeSubsystem.getDeployVelocity().gt(jamThreshold);
      if (jamDebouncer.calculate(isJammed) || unJamTimer.isRunning()) {
        if (!unJamTimer.isRunning()) {
          // new jam, jigle
          unJamTimer.start();
          intakeSubsystem.deploy();
        } else if (unJamTimer.hasElapsed(unjamDuration)) {
          // stop jiggling
          unJamTimer.stop();
          unJamTimer.reset();
          intakeSubsystem.retractForShooting(retractCurrent);
          jamDebouncer.calculate(false); // reset debouncer so we don't immediately re-enter the jammed state
        }
      } else {
        // not jammed, run normally
        intakeSubsystem.retractForShooting(retractCurrent);
      }
    }
  }

}
