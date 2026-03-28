package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShootingConstants.HUB_SETPOINTS_BY_DISTANCE_METERS;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.Function;
import java.util.function.Supplier;

/*
 * Command to shoot fuel without aiming. It will shoot at a fixed yaw, pitch, and velocity
 */
public class ShootAtTargetCommand extends Command {
  private final IndexerSubsystem indexerSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final CommandSwerveDrivetrain drivetrain;
  private final IntakeSubsytem intakeSubsytem;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Function<Translation2d, Translation2d> targetTranslationSelector;
  private final SwerveRequest.FieldCentricFacingAngle swerveRequestFacing = new SwerveRequest.FieldCentricFacingAngle()
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo)
      .withVelocityX(0.0)
      .withVelocityY(0.0);

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
      CommandSwerveDrivetrain drivetrain,
      IntakeSubsytem intakeSubsytem,
      Supplier<Pose2d> robotPoseSupplier,
      Function<Translation2d, Translation2d> targetTranslationSelector) {
    this.feederSubsystem = feederSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.drivetrain = drivetrain;
    this.intakeSubsytem = intakeSubsytem;
    this.robotPoseSupplier = robotPoseSupplier;
    this.targetTranslationSelector = targetTranslationSelector;

    addRequirements(feederSubsystem, indexerSubsystem, shooterSubsystem, drivetrain, intakeSubsytem);
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
    var targetTranslation = targetTranslationSelector.apply(robotPose.getTranslation());

    var targetRotation = targetTranslation.minus(robotPose.getTranslation()).getAngle();
    drivetrain.setControl(swerveRequestFacing.withTargetDirection(targetRotation));

    var targetDistance = targetTranslation.getDistance(robotPose.getTranslation());

    var shooterAngularVelocity = RotationsPerSecond.of(HUB_SETPOINTS_BY_DISTANCE_METERS.get(targetDistance));

    shooterSubsystem.setFlywheelSpeed(shooterAngularVelocity);
    /*
     * Checks to make sure the shooter is ready and up to speed
     * before runnig the indexer and feeder
     */
    if (isShooting || shooterSubsystem.isReadyToShoot()) {
      drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
      isShooting = true;
      feederSubsystem.feedShooter();
      indexerSubsystem.feedShooter();
      intakeSubsytem.retract();
      intakeSubsytem.runIntakeSlow();
    }
  }

  public void end(boolean interrupted) {
    indexerSubsystem.stop();
    feederSubsystem.stop();
    shooterSubsystem.stop();
  }
}
