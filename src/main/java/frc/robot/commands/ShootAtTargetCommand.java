package frc.robot.commands;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShootingConstants.AIM_TOLERANCE;
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
 * Command to aim at a target and shoot fuel. This command will run until interrupted.
 */
public class ShootAtTargetCommand extends Command {
  private final IndexerSubsystem indexerSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final CommandSwerveDrivetrain drivetrain;
  private final IntakeSubsytem intakeSubsytem;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Function<Translation2d, Translation2d> targetTranslationSelector;
  private final SwerveRequest.SwerveDriveBrake swerveDriveBrake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.FieldCentricFacingAngle swerveRequestFacing = new SwerveRequest.FieldCentricFacingAngle()
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo)
      .withVelocityX(0.0)
      .withVelocityY(0.0);

  private boolean isShooting = false;

  /**
   * Constructor for ShootAtTargetCommand
   * 
   * @param indexerSubsystem the indexer subsystem
   * @param feederSubsystem the feeder subsystem
   * @param shooterSubsystem the shooter subsystem
   * @param drivetrain the drivetrain subsystem
   * @param intakeSubsytem the intake subsystem
   * @param robotPoseSupplier the supplier for the robot's pose
   * @param targetTranslationSelector the function to select the target translation
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
    var robotPose = robotPoseSupplier.get();
    var targetTranslation = targetTranslationSelector.apply(robotPose.getTranslation());
    var headingToTarget = targetTranslation.minus(robotPose.getTranslation()).getAngle();
    drivetrain.setControl(swerveRequestFacing.withTargetDirection(headingToTarget));

    var targetDistance = targetTranslation.getDistance(robotPose.getTranslation());
    var shooterAngularVelocity = RotationsPerSecond.of(HUB_SETPOINTS_BY_DISTANCE_METERS.get(targetDistance));
    shooterSubsystem.setFlywheelSpeed(shooterAngularVelocity);

    // Check to make sure the shooter is ready and the drivetrain is aimed before shooting
    var aimError = Math.abs(headingToTarget.minus(robotPose.getRotation()).getRadians());
    if (isShooting || (shooterSubsystem.isReadyToShoot() && aimError <= AIM_TOLERANCE.in(Radian))) {
      isShooting = true;
      drivetrain.setControl(swerveDriveBrake);
      feederSubsystem.feedShooter();
      indexerSubsystem.feedShooter();
      intakeSubsytem.retractForShooting();
    }
  }

  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    drivetrain.setControl(new SwerveRequest.Idle());
    feederSubsystem.stop();
    indexerSubsystem.stop();
    intakeSubsytem.stop();
  }
}
