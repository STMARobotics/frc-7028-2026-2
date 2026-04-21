package frc.robot.commands;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.Constants.ShooterConstants.SHOOTER_OFFSET_ANGLE;
import static frc.robot.Constants.ShootingConstants.AIM_TOLERANCE;
import static frc.robot.Constants.ShootingConstants.HEADING_P;
import static frc.robot.Constants.ShootingConstants.HUB_SETPOINTS_BY_DISTANCE_METERS;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.LEDSubsystem;
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
  private final IntakeSubsytem intakeSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Function<Translation2d, Translation2d> targetTranslationSelector;
  private final IntakeShootingSequence intakeShootingSequence;

  private final SwerveRequest.SwerveDriveBrake swerveDriveBrake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.FieldCentricFacingAngle swerveRequestFacing = new SwerveRequest.FieldCentricFacingAngle()
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
      .withHeadingPID(HEADING_P, 0, 0)
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo)
      .withVelocityX(0.0)
      .withVelocityY(0.0);

  // LED patterns for shooting
  private final LEDPattern shootingPatternTwo = LEDPattern
      .gradient(GradientType.kDiscontinuous, Color.kBlack, Color.kRed)
      .scrollAtRelativeSpeed(Percent.per(Second).of(300));
  private final LEDPattern shootingPatternOne = shootingPatternTwo.reversed();
  private final MutAngularVelocity shooterAngularVelocity = RotationsPerSecond.mutable(0);

  private final Timer shootingTimer = new Timer();
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
      LEDSubsystem ledSubsystem,
      Supplier<Pose2d> robotPoseSupplier,
      Function<Translation2d, Translation2d> targetTranslationSelector) {
    this.feederSubsystem = feederSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.drivetrain = drivetrain;
    this.intakeSubsystem = intakeSubsytem;
    this.ledSubsystem = ledSubsystem;
    this.robotPoseSupplier = robotPoseSupplier;
    this.targetTranslationSelector = targetTranslationSelector;
    this.intakeShootingSequence = new IntakeShootingSequence(intakeSubsytem);

    addRequirements(feederSubsystem, indexerSubsystem, shooterSubsystem, drivetrain, intakeSubsytem, ledSubsystem);
  }

  @Override
  public void initialize() {
    isShooting = false;
    shootingTimer.stop();
    shootingTimer.reset();
    feederSubsystem.stop();
    indexerSubsystem.stop();
    intakeSubsystem.stop();
    shooterSubsystem.stop();
    intakeShootingSequence.reset();
  }

  @Override
  public void execute() {
    var robotPose = robotPoseSupplier.get();
    var targetTranslation = targetTranslationSelector.apply(robotPose.getTranslation());
    var headingToTarget = targetTranslation.minus(robotPose.getTranslation()).getAngle().minus(SHOOTER_OFFSET_ANGLE);

    var targetDistance = targetTranslation.getDistance(robotPose.getTranslation());
    shooterAngularVelocity.mut_replace(HUB_SETPOINTS_BY_DISTANCE_METERS.get(targetDistance), RotationsPerSecond);
    shooterSubsystem.setFlywheelSpeed(shooterAngularVelocity);

    // Check to make sure the shooter is ready and the drivetrain is aimed before shooting
    var aimError = Math.abs(headingToTarget.minus(robotPose.getRotation()).getRadians());
    var isAimReady = aimError <= AIM_TOLERANCE.in(Radians);
    var isShooterReady = shooterSubsystem.isFlywheelAtSpeed();
    if (isShooting || (isShooterReady && isAimReady)) {
      isShooting = true;
      intakeShootingSequence.execute();
      drivetrain.setControl(swerveDriveBrake);
      feederSubsystem.feedShooter();
      indexerSubsystem.feedShooter();
      ledSubsystem.runPatternOnHalves(shootingPatternOne, shootingPatternTwo);
    } else {
      drivetrain.setControl(swerveRequestFacing.withTargetDirection(headingToTarget));
      ledSubsystem.runPattern(LEDSubsystem.ledSegments(Color.kGreen, () -> isAimReady, () -> isShooterReady));
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    drivetrain.setControl(new SwerveRequest.Idle());
    feederSubsystem.stop();
    indexerSubsystem.stop();
    intakeSubsystem.stop();
    ledSubsystem.off();
    shootingTimer.stop();
  }
}