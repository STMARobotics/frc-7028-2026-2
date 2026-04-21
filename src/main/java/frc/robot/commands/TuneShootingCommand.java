package frc.robot.commands;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static frc.robot.Constants.FeederConstants.FEEDER_FEED_VELOCITY;
import static frc.robot.Constants.IndexerConstants.INDEXER_FEED_VELOCITY;
import static frc.robot.Constants.IntakeConstants.DEPLOY_SHOOTING_CURRENT;
import static frc.robot.Constants.ShootingConstants.HUB_BLUE;
import static frc.robot.Constants.ShootingConstants.HUB_RED;
import static frc.robot.Constants.ShootingConstants.HUB_SETPOINTS_BY_DISTANCE_METERS;
import static frc.robot.Constants.ShootingConstants.JAM_DEBOUNCE_TIME;
import static frc.robot.Constants.ShootingConstants.JAM_THRESHOLD;
import static frc.robot.Constants.ShootingConstants.RETRACT_INTAKE_DELAY;
import static frc.robot.Constants.ShootingConstants.UNJAM_DURATION;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.Supplier;

/**
 * Testing command for tuning shots. This is not intended to be used in-game. This command reads
 * from the NetworkTables to get shooter pitch, velocity, and yaw.
 */
public class TuneShootingCommand extends Command {

  private final ShooterSubsystem shooterSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final Supplier<Pose2d> poseSupplier;
  private final IntakeSubsytem intakeSubsytem;
  private final IntakeShootingSequence intakeShootingSequence;

  private final DoubleEntry flywheelSubscriber;
  private final DoubleEntry feederVelocitySubscriber;
  private final DoubleEntry indexerVelocitySubscriber;
  private final DoubleEntry retractDelaySubscriber;
  private final DoubleEntry retractCurrentSubscriber;
  private final DoubleEntry jamDebounceSubscriber;
  private final DoubleEntry unjamDurationSubscriber;
  private final DoubleEntry jamThresholdSubscriber;
  private final DoublePublisher distancePublisher;

  private boolean shooting = false;
  private Translation2d hubTranslation;

  private MutAngularVelocity flywheelVelocity = RotationsPerSecond.mutable(0);
  private MutAngularVelocity feederVelocity = RotationsPerSecond.mutable(0);
  private MutAngularVelocity indexerVelocity = RotationsPerSecond.mutable(0);
  private MutTime retractDelay = Seconds.mutable(0);
  private MutCurrent retractCurrent = Amps.mutable(0);
  private MutTime jamDebounceTime = Seconds.mutable(0);
  private MutTime unjamDuration = Seconds.mutable(0);
  private MutAngularVelocity jamThreshold = RotationsPerSecond.mutable(0);

  public TuneShootingCommand(
      IndexerSubsystem indexerSubsystem,
      FeederSubsystem feederSubsystem,
      ShooterSubsystem shooterSubsystem,
      LEDSubsystem ledSubsystem,
      IntakeSubsytem intakeSubsytem,
      Supplier<Pose2d> poseSupplier) {

    this.indexerSubsystem = indexerSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsytem = intakeSubsytem;
    this.poseSupplier = poseSupplier;
    this.intakeShootingSequence = new IntakeShootingSequence(intakeSubsytem);

    var nt = NetworkTableInstance.getDefault();
    var table = nt.getTable("Tune Shoot");
    distancePublisher = table.getDoubleTopic("_Hub Distance").publish();
    flywheelSubscriber = table.getDoubleTopic("Flywheel Velocity (RPS)").getEntry(0.0);
    flywheelSubscriber.set(HUB_SETPOINTS_BY_DISTANCE_METERS.get(1.0));
    feederVelocitySubscriber = table.getDoubleTopic("Feeder Velocity (RPS)")
        .getEntry(FEEDER_FEED_VELOCITY.in(RotationsPerSecond));
    feederVelocitySubscriber.set(FEEDER_FEED_VELOCITY.in(RotationsPerSecond));
    indexerVelocitySubscriber = table.getDoubleTopic("Indexer Velocity (RPS)")
        .getEntry(INDEXER_FEED_VELOCITY.in(RotationsPerSecond));
    indexerVelocitySubscriber.set(INDEXER_FEED_VELOCITY.in(RotationsPerSecond));
    retractDelaySubscriber = table.getDoubleTopic("Retract Delay (s)").getEntry(0.0);
    retractDelaySubscriber.set(RETRACT_INTAKE_DELAY.in(Seconds));
    retractCurrentSubscriber = table.getDoubleTopic("Retract Current (A)").getEntry(0.0);
    retractCurrentSubscriber.set(DEPLOY_SHOOTING_CURRENT.in(Amps));
    jamDebounceSubscriber = table.getDoubleTopic("Jam Debounce Time (s)").getEntry(0.0);
    jamDebounceSubscriber.set(JAM_DEBOUNCE_TIME.in(Seconds));
    unjamDurationSubscriber = table.getDoubleTopic("Unjam Duration (s)").getEntry(0.0);
    unjamDurationSubscriber.set(UNJAM_DURATION.in(Seconds));
    jamThresholdSubscriber = table.getDoubleTopic("Jam Threshold (RPS)").getEntry(0.0);
    jamThresholdSubscriber.set(JAM_THRESHOLD.in(RotationsPerSecond));

    addRequirements(indexerSubsystem, shooterSubsystem, feederSubsystem, intakeSubsytem);
  }

  @Override
  public void initialize() {
    shooting = false;
    var alliance = DriverStation.getAlliance();
    hubTranslation = (alliance.isEmpty() || alliance.get() == Blue) ? HUB_BLUE : HUB_RED;
    intakeShootingSequence.reset(jamDebounceTime.mut_replace(jamDebounceSubscriber.get(), Seconds));
  }

  @Override
  public void execute() {
    var distanceToHub = poseSupplier.get().getTranslation().getDistance(hubTranslation);
    distancePublisher.accept(distanceToHub);

    shooterSubsystem.setFlywheelSpeed(flywheelVelocity.mut_replace(flywheelSubscriber.get(), RotationsPerSecond));
    if (shooting || shooterSubsystem.isFlywheelAtSpeed()) {
      shooting = true;

      intakeShootingSequence.execute(
          retractDelay.mut_replace(retractDelaySubscriber.get(), Seconds),
            retractCurrent.mut_replace(retractCurrentSubscriber.get(), Amps),
            jamThreshold.mut_replace(jamThresholdSubscriber.get(), RotationsPerSecond),
            unjamDuration.mut_replace(unjamDurationSubscriber.get(), Seconds));
      feederSubsystem.runFeeder(feederVelocity.mut_replace(feederVelocitySubscriber.get(), RotationsPerSecond));
      indexerSubsystem.runIndexer(indexerVelocity.mut_replace(indexerVelocitySubscriber.get(), RotationsPerSecond));
    }
  }

  @Override
  public void end(boolean interrupted) {
    feederSubsystem.stop();
    indexerSubsystem.stop();
    shooterSubsystem.stop();
    intakeSubsytem.stop();
  }
}