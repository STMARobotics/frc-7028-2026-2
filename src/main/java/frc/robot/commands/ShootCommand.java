package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.Constants.ShootingConstants.HUB_SETPOINTS_BY_DISTANCE_METERS;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
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

/*
 * Command to shoot fuel without aiming. It will shoot at a fixed velocity
 */
public class ShootCommand extends Command {
  private final IndexerSubsystem indexerSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final CommandSwerveDrivetrain drivetrain;
  private final IntakeSubsytem intakeSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final AngularVelocity shooterAngularVelocity;
  private final SwerveRequest.SwerveDriveBrake swerveBrakeRequest = new SwerveRequest.SwerveDriveBrake();

  // LED patterns for shooting
  private final LEDPattern patternTwo = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlack, Color.kBlue)
      .scrollAtRelativeSpeed(Percent.per(Second).of(300));
  private final LEDPattern patternOne = patternTwo.reversed();

  private final IntakeShootingSequence intakeShootingSequence;
  private final Timer shootingTimer = new Timer();
  private boolean isShooting = false;

  /**
   * Constructor for ShootCommand
   * 
   * @param indexerSubsystem the indexer subsystem
   * @param feederSubsystem the feeder subsystem
   * @param shooterSubsystem the shooter subsystem
   * @param drivetrain the drivetrain subsystem
   * @param intakeSubsytem the intake subsystem
   * @param ledSubsystem the LED subsystem
   * @param targetDistance the distance to the target to use for velocity lookup
   */
  public ShootCommand(
      IndexerSubsystem indexerSubsystem,
      FeederSubsystem feederSubsystem,
      ShooterSubsystem shooterSubsystem,
      CommandSwerveDrivetrain drivetrain,
      IntakeSubsytem intakeSubsytem,
      LEDSubsystem ledSubsystem,
      Distance targetDistance) {
    this.feederSubsystem = feederSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.drivetrain = drivetrain;
    this.intakeSubsystem = intakeSubsytem;
    this.ledSubsystem = ledSubsystem;
    this.intakeShootingSequence = new IntakeShootingSequence(intakeSubsytem);

    shooterAngularVelocity = RotationsPerSecond.of(HUB_SETPOINTS_BY_DISTANCE_METERS.get(targetDistance.in(Meters)));

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
    shooterSubsystem.setFlywheelSpeed(shooterAngularVelocity);
    drivetrain.setControl(swerveBrakeRequest);

    // Check to make sure the shooter is ready before running the indexer and feeder
    if (isShooting || shooterSubsystem.isFlywheelAtSpeed()) {
      isShooting = true;
      shootingTimer.start();
      if (shootingTimer.hasElapsed(0.06)) {
        indexerSubsystem.feedShooter();
      }
      if (shootingTimer.hasElapsed(0.06)) {
        intakeShootingSequence.execute();
      }
      feederSubsystem.feedShooter();
      ledSubsystem.runPatternOnHalves(patternOne, patternTwo);
    } else {
      ledSubsystem.off();
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
