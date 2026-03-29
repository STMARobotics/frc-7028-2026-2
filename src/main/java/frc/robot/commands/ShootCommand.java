package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShootingConstants.HUB_SETPOINTS_BY_DISTANCE_METERS;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.ShooterSubsystem;

/*
 * Command to shoot fuel without aiming. It will shoot at a fixed velocity
 */
public class ShootCommand extends Command {
  private final IndexerSubsystem indexerSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final CommandSwerveDrivetrain drivetrain;
  private final IntakeSubsytem intakeSubsytem;
  private final AngularVelocity shooterAngularVelocity;
  private final SwerveRequest.SwerveDriveBrake swerveBrakeRequest = new SwerveRequest.SwerveDriveBrake();

  private boolean isShooting = false;

  /**
   * Constructor for ShootCommand
   * 
   * @param indexerSubsystem the indexer subsystem
   * @param feederSubsystem the feeder subsystem
   * @param shooterSubsystem the shooter subsystem
   * @param drivetrain the drivetrain subsystem
   * @param intakeSubsytem the intake subsystem
   * @param targetDistance the distance to the target to use for velocity lookup
   */
  public ShootCommand(
      IndexerSubsystem indexerSubsystem,
      FeederSubsystem feederSubsystem,
      ShooterSubsystem shooterSubsystem,
      CommandSwerveDrivetrain drivetrain,
      IntakeSubsytem intakeSubsytem,
      Distance targetDistance) {
    this.feederSubsystem = feederSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.drivetrain = drivetrain;
    this.intakeSubsytem = intakeSubsytem;

    shooterAngularVelocity = RotationsPerSecond.of(HUB_SETPOINTS_BY_DISTANCE_METERS.get(targetDistance.in(Meters)));

    addRequirements(feederSubsystem, indexerSubsystem, shooterSubsystem, drivetrain, intakeSubsytem);
  }

  @Override
  public void initialize() {
    isShooting = false;
  }

  @Override
  public void execute() {
    shooterSubsystem.setFlywheelSpeed(shooterAngularVelocity);
    drivetrain.setControl(swerveBrakeRequest);

    // Check to make sure the shooter is ready before running the indexer and feeder
    if (isShooting || shooterSubsystem.isReadyToShoot()) {
      isShooting = true;
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
