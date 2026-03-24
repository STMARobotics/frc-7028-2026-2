package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static frc.robot.Constants.FeederConstants.FEEDER_FEED_VELOCITY;
import static frc.robot.Constants.SpindexerConstants.SPINDEXER_FEED_VELOCITY;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShootingConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LEDSubsystemContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import java.util.function.Supplier;

/**
 * Testing command for tuning shots. This is not intended to be used in-game. This command reads
 * from the NetworkTables to get shooter pitch, velocity, and yaw.
 */
public class TuneShootingCommand extends Command {

  private final ShooterSubsystem shooterSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final SpindexerSubsystem spindexerSubsystem;
  private final Supplier<Pose2d> poseSupplier;

  private final DoubleEntry pitchSubscriber;
  private final DoubleEntry flywheelSubscriber;
  private final DoubleEntry yawSubscriber;
  private final DoubleEntry feederVelocitySubscriber;
  private final DoubleEntry spindexerVelocitySubscriber;
  private final DoublePublisher distancePublisher;

  private boolean shooting = false;
  private Translation2d hubTranslation;

  private MutAngle pitchMeasure = Radian.mutable(0);
  private MutAngle yawMeasure = Radian.mutable(0);
  private MutAngularVelocity topVelocityMeasure = RotationsPerSecond.mutable(0);
  private MutAngularVelocity feederVelocityMeasure = RotationsPerSecond.mutable(0);
  private MutAngularVelocity spindexerVelocityMeasure = RotationsPerSecond.mutable(0);

  public TuneShootingCommand(
      SpindexerSubsystem spindexerSubsystem,
      FeederSubsystem feederSubsystem,
      ShooterSubsystem shooterSubsystem,
      LEDSubsystemContainer ledSubsystem,
      Supplier<Pose2d> poseSupplier) {

    this.spindexerSubsystem = spindexerSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.poseSupplier = poseSupplier;

    var nt = NetworkTableInstance.getDefault();
    var table = nt.getTable("Tune Shoot");
    distancePublisher = table.getDoubleTopic("Hub Distance").publish();
    pitchSubscriber = table.getDoubleTopic("Pitch (degrees)").getEntry(0.0);
    pitchSubscriber.set(0.0);
    flywheelSubscriber = table.getDoubleTopic("Flywheel Velocity (RPS)").getEntry(0.0);
    flywheelSubscriber.set(0.0);
    feederVelocitySubscriber = table.getDoubleTopic("Feeder Velocity (RPS)")
        .getEntry(FEEDER_FEED_VELOCITY.in(RotationsPerSecond));
    feederVelocitySubscriber.set(FEEDER_FEED_VELOCITY.in(RotationsPerSecond));
    yawSubscriber = table.getDoubleTopic("Yaw (Degrees)").getEntry(180.0);
    yawSubscriber.set(180.0);
    spindexerVelocitySubscriber = table.getDoubleTopic("Spindexer Velocity (RPS)")
        .getEntry(SPINDEXER_FEED_VELOCITY.in(RotationsPerSecond));
    spindexerVelocitySubscriber.set(SPINDEXER_FEED_VELOCITY.in(RotationsPerSecond));

    addRequirements(spindexerSubsystem, shooterSubsystem, feederSubsystem);
  }

  @Override
  public void initialize() {
    shooting = false;
    var alliance = DriverStation.getAlliance();
    hubTranslation = (alliance.isEmpty() || alliance.get() == Blue) ? ShootingConstants.TARGET_BLUE
        : ShootingConstants.TARGET_RED;
  }

  @Override
  public void execute() {
    var turretDistanceToHub = ShooterSubsystem.getShooterTranslation(poseSupplier.get()).getDistance(hubTranslation);
    distancePublisher.accept(turretDistanceToHub);

    shooterSubsystem.setPitchAngle(pitchMeasure.mut_replace(pitchSubscriber.get(0.0), Degrees));
    shooterSubsystem.setYawAngle(yawMeasure.mut_replace(yawSubscriber.get(180.0), Degrees));
    shooterSubsystem.setFlywheelSpeed(topVelocityMeasure.mut_replace(flywheelSubscriber.get(0.0), RotationsPerSecond));
    if (shooting || (shooterSubsystem.isFlywheelAtSpeed() && shooterSubsystem.isPitchAtSetpoint())) {
      feederSubsystem
          .runFeeder(feederVelocityMeasure.mut_replace(feederVelocitySubscriber.get(0.0), RotationsPerSecond));
      spindexerSubsystem
          .runSpindexer(spindexerVelocityMeasure.mut_replace(spindexerVelocitySubscriber.get(0.0), RotationsPerSecond));
      shooting = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    feederSubsystem.stop();
    spindexerSubsystem.stop();
    shooterSubsystem.stopAll();
  }
}