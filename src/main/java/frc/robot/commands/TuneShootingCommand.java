package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static frc.robot.Constants.FeederConstants.FEEDER_FEED_VELOCITY;
import static frc.robot.Constants.IndexerConstants.INDEXER_FEED_VELOCITY;
import static frc.robot.Constants.ShootingConstants.DEPLOY_INTAKE_TIME;
import static frc.robot.Constants.ShootingConstants.RETRACT_INTAKE_TIME;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShootingConstants;
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

  private final DoubleEntry flywheelSubscriber;
  private final DoubleEntry feederVelocitySubscriber;
  private final DoubleEntry indexerVelocitySubscriber;
  private final DoublePublisher distancePublisher;

  private final Timer shootingTimer = new Timer();

  private boolean shooting = false;
  private boolean retractIntake = false;
  private Translation2d hubTranslation;

  private MutAngularVelocity topVelocityMeasure = RotationsPerSecond.mutable(0);
  private MutAngularVelocity feederVelocityMeasure = RotationsPerSecond.mutable(0);
  private MutAngularVelocity indexerVelocityMeasure = RotationsPerSecond.mutable(0);

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

    var nt = NetworkTableInstance.getDefault();
    var table = nt.getTable("Tune Shoot");
    distancePublisher = table.getDoubleTopic("Hub Distance").publish();
    flywheelSubscriber = table.getDoubleTopic("Flywheel Velocity (RPS)").getEntry(0.0);
    flywheelSubscriber.set(0.0);
    feederVelocitySubscriber = table.getDoubleTopic("Feeder Velocity (RPS)")
        .getEntry(FEEDER_FEED_VELOCITY.in(RotationsPerSecond));
    feederVelocitySubscriber.set(FEEDER_FEED_VELOCITY.in(RotationsPerSecond));
    indexerVelocitySubscriber = table.getDoubleTopic("Indexer Velocity (RPS)")
        .getEntry(INDEXER_FEED_VELOCITY.in(RotationsPerSecond));
    indexerVelocitySubscriber.set(INDEXER_FEED_VELOCITY.in(RotationsPerSecond));

    addRequirements(indexerSubsystem, shooterSubsystem, feederSubsystem, intakeSubsytem);
  }

  @Override
  public void initialize() {
    shooting = false;
    var alliance = DriverStation.getAlliance();
    hubTranslation = (alliance.isEmpty() || alliance.get() == Blue) ? ShootingConstants.HUB_BLUE
        : ShootingConstants.HUB_RED;
    shootingTimer.stop();
    shootingTimer.reset();
    retractIntake = false;
  }

  @Override
  public void execute() {
    var distanceToHub = poseSupplier.get().getTranslation().getDistance(hubTranslation);
    distancePublisher.accept(distanceToHub);

    shooterSubsystem.setFlywheelSpeed(topVelocityMeasure.mut_replace(flywheelSubscriber.get(0.0), RotationsPerSecond));
    if (shooting || shooterSubsystem.isFlywheelAtSpeed()) {
      shooting = true;
      shootingTimer.start();
      if (retractIntake) {
        retractIntake = !shootingTimer.advanceIfElapsed(DEPLOY_INTAKE_TIME.in(Seconds));
        intakeSubsytem.retractForShooting();
      } else {
        retractIntake = shootingTimer.advanceIfElapsed(RETRACT_INTAKE_TIME.in(Seconds));
        intakeSubsytem.deploy();
      }
      intakeSubsytem.runIntakeForShooting();
      feederSubsystem
          .runFeeder(feederVelocityMeasure.mut_replace(feederVelocitySubscriber.get(0.0), RotationsPerSecond));
      indexerSubsystem
          .runIndexer(indexerVelocityMeasure.mut_replace(indexerVelocitySubscriber.get(0.0), RotationsPerSecond));
    }
  }

  @Override
  public void end(boolean interrupted) {
    feederSubsystem.stop();
    indexerSubsystem.stop();
    shooterSubsystem.stop();
    intakeSubsytem.stop();
    shootingTimer.stop();
  }
}