package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.ShootingConstants.HUB_SETPOINTS_BY_DISTANCE_METERS;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSetpoints;
import frc.robot.subsystems.IndexerSubsystem;

/*
 * Command to shoot fuel without aiming. It will shoot at a fixed yaw, pitch, and velocity
 */
public class ShootCommand extends Command {
  private final IndexerSubsystem indexerSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ShooterSetpoints setpoints;
  private boolean isShooting = false;

  /**
   * Constructor for ShootCommand
   * 
   * @param indexerSubsystem the indexer subsystem
   * @param feederSubsystem the feeder subsystem
   * @param shooterSubsystem the shooter subsystem
   * @param targetDistance the distance of the target to use for setpoints
   */
  public ShootCommand(
      IndexerSubsystem indexerSubsystem,
      FeederSubsystem feederSubsystem,
      ShooterSubsystem shooterSubsystem,
      Distance targetDistance) {
    this.feederSubsystem = feederSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    setpoints = HUB_SETPOINTS_BY_DISTANCE_METERS.get(targetDistance.in(Meters));

    addRequirements(feederSubsystem, indexerSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    isShooting = false;
  }

  @Override
  public void execute() {
    /*
     * sets the Yaw, Pitch, and Angle
     */
    shooterSubsystem.setYawAngle(Degrees.of(180));
    shooterSubsystem.setPitchAngle(setpoints.targetPitch());
    shooterSubsystem.setFlywheelSpeed(setpoints.targetFlywheelSpeed());
    /*
     * Checks to make sure the shooter is ready and up to speed
     * before runnig the indexer and feeder
     */
    if (isShooting || shooterSubsystem.isReadyToShoot()) {
      isShooting = true;
      indexerSubsystem.runIndexer(setpoints.indexerVelocity());
      feederSubsystem.runFeeder(setpoints.feederVelocity());
    }
  }

  public void end(boolean interrupted) {
    indexerSubsystem.stop();
    feederSubsystem.stop();
    shooterSubsystem.stopAll();
  }
}
