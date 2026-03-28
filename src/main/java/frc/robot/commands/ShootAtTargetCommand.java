package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShootingConstants.HUB_SETPOINTS_BY_DISTANCE_METERS;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/*
 * Command to shoot fuel without aiming. It will shoot at a fixed yaw, pitch, and velocity
 */
public class ShootAtTargetCommand extends Command {
  private final IndexerSubsystem indexerSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final AngularVelocity shooterAngularVelocity;
  private final CommandSwerveDrivetrain drivetrain;
  private final Supplier<Pose2d> robotPoseSupplier; 
  private final Supplier<Translation2d> targetTranslationSupplier;

  private boolean isShooting = false;

  /**
   * Constructor for ShootCommand
   * 
   * @param indexerSubsystem the indexer subsystem
   * @param feederSubsystem the feeder subsystem
   * @param shooterSubsystem the shooter subsystem
   * @param targetDistance the distance of the target to use for setpoints
   */
  public ShootAtTargetCommand(
      IndexerSubsystem indexerSubsystem,
      FeederSubsystem feederSubsystem,
      ShooterSubsystem shooterSubsystem,
      Distance targetDistance,
      CommandSwerveDrivetrain drivetrain,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Translation2d> targetTranslationSupplier ) {
    this.feederSubsystem = feederSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.drivetrain = drivetrain;
    this.robotPoseSupplier = robotPoseSupplier;
    this.targetTranslationSupplier = targetTranslationSupplier;

    shooterAngularVelocity = RotationsPerSecond.of(HUB_SETPOINTS_BY_DISTANCE_METERS.get(targetDistance.in(Meters)));

    addRequirements(feederSubsystem, indexerSubsystem, shooterSubsystem, drivetrain);
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
  var robotPose = robotPoseSupplier.get();
  var targetTranslation = targetTranslationSupplier.get();

  var angleToTarget = targetTranslation.minus(robotPose.getTranslation()).getAngle();
  

    shooterSubsystem.setFlywheelSpeed(shooterAngularVelocity);
    /*
     * Checks to make sure the shooter is ready and up to speed
     * before runnig the indexer and feeder
     */
    if (isShooting || shooterSubsystem.isReadyToShoot()) {
      drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
      isShooting = true;
      // TODO run indexer and feeder
        
    }
  }

  public void end(boolean interrupted) {
    indexerSubsystem.stop();
    feederSubsystem.stop();
    shooterSubsystem.stop();
  }
}
