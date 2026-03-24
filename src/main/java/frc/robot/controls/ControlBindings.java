package frc.robot.controls;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Abstract class that defines the available bindings for driver controls. It can be extended to provide a "control
 * scheme"
 */
public abstract class ControlBindings {

  /**
   * Supplier for the driver desired X speed
   *
   * @return velocity supplier
   */
  public abstract Supplier<LinearVelocity> translationX();

  /**
   * Supplier for the driver desired Y speed
   *
   * @return velocity supplier
   */
  public abstract Supplier<LinearVelocity> translationY();

  /**
   * Supplier for the drive desired omega rotation
   *
   * @return velocity supplier
   */
  public abstract Supplier<AngularVelocity> omega();

  /**
   * Triggers putting the wheels into an X configuration
   *
   * @return optional trigger
   */
  public Optional<Trigger> wheelsToX() {
    return Optional.empty();
  }

  /**
   * Trigger to reset the robot position to the current alliance stations righthand corner, facing downfield.
   * <p>
   * This is only intended to be used for practice, not in competition
   * 
   * @return optional trigger
   */
  public Optional<Trigger> resetFieldPosition() {
    return Optional.empty();
  }

  /**
   * Trigger to reset the robot position using AprilTags on the field.
   * <p>
   * This is only intended to be used for practice, not in competition
   * 
   * @return optional trigger
   */
  public Optional<Trigger> resetFieldPositionFromAprilTags() {
    return Optional.empty();
  }

  /**
   * Trigger for running the intake
   *
   * @return optional trigger
   */
  public Optional<Trigger> runIntake() {
    return Optional.empty();
  }

  /**
   * Trigger for stopping the intake
   *
   * @return optional trigger
   */
  public Optional<Trigger> stopIntake() {
    return Optional.empty();
  }

  /**
   * Trigger for ejecting game piece out of the intake
   *
   * @return optional trigger
   */
  public Optional<Trigger> eject() {
    return Optional.empty();
  }

  /**
   * Trigger for deploying the intake
   *
   * @return optional trigger
   */
  public Optional<Trigger> deployIntake() {
    return Optional.empty();
  }

  /**
   * Trigger for retracting the intake
   *
   * @return optional trigger
   */
  public Optional<Trigger> retractIntake() {
    return Optional.empty();
  }

  /**
   * Trigger for manual shooting
   *
   * @return optional trigger
   */
  public Optional<Trigger> manualShoot() {
    return Optional.empty();
  }

  /**
   * Trigger for automatic shooting
   *
   * @return optional trigger
   */
  public Optional<Trigger> autoShoot() {
    return Optional.empty();
  }

  /**
   * Trigger to run the shooting tuning command
   * 
   * @return optional trigger
   */
  public Optional<Trigger> tuneShoot() {
    return Optional.empty();
  }

  /**
   * Trigger to run the shuttling command
   * 
   * @return optional trigger
   */
  public Optional<Trigger> shuttle() {
    return Optional.empty();
  }

}