package frc.robot.commands;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.ShooterConstants.FUEL_EXIT_ANGLE;
import static frc.robot.Constants.ShooterConstants.ROBOT_TO_SHOOTER;
import static frc.robot.Constants.ShootingConstants.AIM_TOLERANCE;
import static frc.robot.Constants.ShootingConstants.DEPLOY_INTAKE_TIME;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_TO_FUEL_VELOCITY_MULTIPLIER;
import static frc.robot.Constants.ShootingConstants.HEADING_P;
import static frc.robot.Constants.ShootingConstants.RETRACT_INTAKE_TIME;
import static frc.robot.Constants.TeleopDriveConstants.CENTER_OF_ROTATION;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

/**
 * This command automatically shoots at a target until interrupted. It takes current robot velocity into account to
 * "shoot on the move". This command must be interrupted to stop, most likely by the driver releasing a button or by an
 * autonomous routine setting an end condition.
 * <p>
 * This command has configurability to be used for both shooting at the hub and shooting while shuttling fuel across the
 * field. The target to shoot at is determined by a function, and the shooter settings lookup table is a parameter.
 */
public class ShootOnTheMoveCommand extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final ShooterSubsystem shooterSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final IntakeSubsytem intakeSubsystem;
  private final LEDSubsystem ledSubsystem;

  private final Supplier<LinearVelocity> translationXSupplier;
  private final Supplier<LinearVelocity> translationYSupplier;
  private final Function<Translation2d, Translation2d> targetSelector;
  private final InterpolatingDoubleTreeMap lookupTable;
  private final SwerveRequest.FieldCentricFacingAngle swerveRequestFacing = new SwerveRequest.FieldCentricFacingAngle()
      .withHeadingPID(HEADING_P, 0, 0)
      .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
      .withCenterOfRotation(CENTER_OF_ROTATION)
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private final LEDPattern shootingPatternTwo = LEDPattern
      .gradient(GradientType.kDiscontinuous, Color.kBlack, Color.kRed)
      .scrollAtRelativeSpeed(Percent.per(Second).of(300));
  private final LEDPattern shootingPatternOne = shootingPatternTwo.reversed();

  // Reusable object to prevent reallocation (to reduce memory pressure)
  private final MutAngularVelocity shooterAngularVelocity = RotationsPerSecond.mutable(0);

  private final Timer shootingTimer = new Timer();
  private boolean isShooting;

  /**
   * Constructor
   * 
   * @param shooterSubsystem shooter subsystem
   * @param feederSubsystem feeder subsystem
   * @param indexerSubsystem spindexer subsystem
   * @param ledSubsystem led subsystem
   * @param robotPoseSupplier robot pose supplier
   * @param robotSpeedSupplier robot speed supplier
   * @param targetSelector function that takes the shooter's translation and returns the target translation to shoot at
   * @param lookupTable lookup table mapping distance to shooter setpoints
   */
  public ShootOnTheMoveCommand(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooterSubsystem,
      FeederSubsystem feederSubsystem,
      IndexerSubsystem indexerSubsystem,
      IntakeSubsytem intakeSubsystem,
      LEDSubsystem ledSubsystem,
      Supplier<LinearVelocity> translationXSupplier,
      Supplier<LinearVelocity> translationYSupplier,
      Function<Translation2d, Translation2d> targetSelector,
      InterpolatingDoubleTreeMap lookupTable) {
    this.drivetrain = drivetrain;
    this.shooterSubsystem = shooterSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.targetSelector = targetSelector;
    this.lookupTable = lookupTable;

    addRequirements(drivetrain, shooterSubsystem, feederSubsystem, indexerSubsystem, intakeSubsystem, ledSubsystem);
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
  }

  @Override
  public void execute() {
    var drivetrainState = drivetrain.getState();
    var robotPose = drivetrainState.Pose;
    var fieldChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrainState.Speeds, robotPose.getRotation());

    // Translation of the shooter on the field (used for distance/angle calculations).
    var shooterTranslation = ShooterSubsystem.getShooterPose(robotPose).getTranslation();

    // Get the target to shoot at
    var targetTranslation = targetSelector.apply(shooterTranslation);
    var actualTargetDistance = shooterTranslation.getDistance(targetTranslation);

    // 1. Compute the velocity of the fuel at the shooter's location on the field.
    //
    // - When the robot (or turret) is moving, the fuel inherits that motion at the moment it leaves the flywheel. To
    // predict where the fuel will land we must account for that initial velocity.
    // - There are two contributions:
    // A) vRobot: the robot's linear translation velocity
    // B) vTan: tangential velocity at the shooter from angular rotation acting on the shooter
    //
    // The sum of these gives the initial velocity of the fuel relative to the field, which we use to compute how far
    // the projectile will shift during its flight.
    var vRobot = new Translation2d(fieldChassisSpeeds.vxMetersPerSecond, fieldChassisSpeeds.vyMetersPerSecond);
    // Total angular rate that affects the shooter (robot yaw + turret yaw rate).
    var omega = fieldChassisSpeeds.omegaRadiansPerSecond;

    // Convert rotation about the robot center into a tangential linear velocity at the shooter.
    // This creates a tangential velocity at the shooter because the shooter is offset from the robot center.
    var robotToShooter = shooterTranslation.minus(robotPose.getTranslation());
    var vTan = new Translation2d(-omega * robotToShooter.getY(), omega * robotToShooter.getX());

    // Effective shooter velocity = robot linear + tangential from rotation.
    var effectiveShooterVelocity = vRobot.plus(vTan);

    // 2. Iteratively solve for the aim point that compensates for time-of-flight.
    //
    // Rationale:
    // - The flight time depends on the flywheel speed and pitch, which are chosen based on the distance to the target.
    // But the distance depends on the offset caused by the robot's motion during flight — a circular dependency.
    // - We therefore perform a few iterations: estimate distance -> choose shooter setpoints -> compute flight time ->
    // predict where the target will appear after that flight -> repeat.
    var predictedTargetTranslation = targetTranslation;
    var targetPredictedOffset = new Translation2d();

    double shooterVelocity;
    for (int i = 0; i < 4; i++) {
      // Distance from the shooter to the currently predicted target point.
      // This distance drives the lookup table which selects pitch and flywheel speed.
      var predictedTargetDistance = shooterTranslation.getDistance(predictedTargetTranslation);
      shooterVelocity = lookupTable.get(predictedTargetDistance);

      var timeUntilScored = 0.0;

      if (Math.abs(shooterVelocity) > 1e-3) { // Avoid divide-by-zero if flywheel is stopped.
        // Approximate time-of-flight using the horizontal component of the fuel's exit velocity. Use actual distance to
        // target (not predicted) for flight time to avoid divergence when moving away from the target.
        //
        // - For compensating lateral translation caused by robot motion, the horizontal (field-parallel) component of
        // the fuel velocity determines how far the fuel travels horizontally during flight. How high the ball arcs
        // doesn't matter, it only matters how fast it moves horizontally.
        // - We approximate flight time as: time = horizontal_distance / (exit_speed * cos(pitch)).
        double fuelExitVelocity = FLYWHEEL_TO_FUEL_VELOCITY_MULTIPLIER * shooterVelocity;
        timeUntilScored = actualTargetDistance / (fuelExitVelocity * Math.cos(FUEL_EXIT_ANGLE.in(Radians)));
      }

      // Compute how far the fuel initial velocity (from robot motion) will shift the target point.
      // We subtract this offset from the real target position to produce a "lead" point to aim at.
      //
      // - If the robot is moving forward, the fuel will land further forward relative to a stationary shot. To hit the
      // desired (static) target we need to aim backwards relative to the instantaneous target position by the amount
      // the fuel will be carried during flight.
      targetPredictedOffset = effectiveShooterVelocity.times(timeUntilScored);
      predictedTargetTranslation = targetTranslation.minus(targetPredictedOffset);
    }

    SmartDashboard.putNumber("Shooting offset X", targetPredictedOffset.getX());
    SmartDashboard.putNumber("Shooting offset Y", targetPredictedOffset.getY());

    // After iterating, resolve final shooter setpoints for the converged predicted target.
    shooterVelocity = lookupTable.get(predictedTargetTranslation.getDistance(shooterTranslation));

    // Compute the heading required to point at the compensated aim point.
    var angleToTarget = predictedTargetTranslation.minus(shooterTranslation)
        .getAngle()
        .rotateBy(ROBOT_TO_SHOOTER.getRotation());
    SmartDashboard.putNumber("Angle To Target", angleToTarget.getRadians());

    // Command the shooter flywheel speed from the lookup table.
    shooterSubsystem.setFlywheelSpeed(shooterAngularVelocity.mut_replace(shooterVelocity, RotationsPerSecond));

    // The control request is from the operator's perspective, but the angle's origin is always from the blue
    // perspective, so it needs to be flipped when the alliance is red
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      angleToTarget = angleToTarget.rotateBy(Rotation2d.kPi);
    }

    // Translate, but rotate at the target.
    drivetrain.setControl(
        swerveRequestFacing.withTargetDirection(angleToTarget)
            .withVelocityX(translationXSupplier.get())
            .withVelocityY(translationYSupplier.get()));

    // Check if all of the conditions are met to be ready to shoot
    var isFlywheelReady = shooterSubsystem.isFlywheelAtSpeed();
    var aimError = Math.abs(angleToTarget.minus(robotPose.getRotation()).getRadians());
    var isAimReady = aimError <= AIM_TOLERANCE.in(Radians);
    if ((isShooting) || (isFlywheelReady && isAimReady)) {
      // All conditions met. Continuously feed until the command is interrupted
      isShooting = true;
      shootingTimer.start();
      if (shootingTimer.hasElapsed(DEPLOY_INTAKE_TIME.in(Seconds))) {
        intakeSubsystem.retractForShooting();
      } else if (shootingTimer.hasElapsed(RETRACT_INTAKE_TIME.in(Seconds))) {
        intakeSubsystem.deploy();
      } else {
        intakeSubsystem.retractForShooting();
      }
      feederSubsystem.feedShooter();
      indexerSubsystem.feedShooter();
      intakeSubsystem.runIntakeForShooting();
      shooterSubsystem.setFlywheelSpeed(shooterAngularVelocity);
      ledSubsystem.runPatternOnHalves(shootingPatternOne, shootingPatternTwo);
    } else {
      ledSubsystem.runPattern(LEDSubsystem.ledSegments(Color.kGreen, () -> isAimReady, () -> isFlywheelReady));
      shooterSubsystem.setFlywheelSpeed(shooterAngularVelocity);
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