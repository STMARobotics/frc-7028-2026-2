package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.FieldConstants.FIELD_WIDTH;
import static frc.robot.Constants.ShootingConstants.HUB_BLUE;
import static frc.robot.Constants.ShootingConstants.HUB_RED;
import static frc.robot.Constants.ShootingConstants.SHUTTLE_BLUE_HIGH;
import static frc.robot.Constants.ShootingConstants.SHUTTLE_BLUE_LOW;
import static frc.robot.Constants.ShootingConstants.SHUTTLE_OFFSET_DISTANCE;
import static frc.robot.Constants.ShootingConstants.SHUTTLE_RED_HIGH;
import static frc.robot.Constants.ShootingConstants.SHUTTLE_RED_LOW;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_ANGULAR_VELOCITY;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_VELOCITY;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ShootAtTargetCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.Supplier;

/**
 * Factory class for creating commands with the necessary subsystem dependencies.
 */
public class CommandFactory {
  private final CommandSwerveDrivetrain drivetrainSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final IntakeSubsytem intakeSubsystem;
  private final LEDSubsystem ledSubsystem;

  /**
   * Constructor for Commandfactory, takes in all subsystems as parameters to use in command creation methods.
   * 
   * @param drivetrainSubsystem drivetrain subsystem
   * @param shooterSubsystem shooter subsystem
   * @param indexerSubsystem indexer subsystem
   * @param feederSubsystem feeder subsystem
   * @param intakeSubsystem intake subsystem
   * @param ledSubsystem led subsystem
   */
  public CommandFactory(
      CommandSwerveDrivetrain drivetrainSubsystem,
      ShooterSubsystem shooterSubsystem,
      IndexerSubsystem indexerSubsystem,
      FeederSubsystem feederSubsystem,
      IntakeSubsytem intakeSubsystem,
      LEDSubsystem ledSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.ledSubsystem = ledSubsystem;
  }

  /**
   * Creates a command to shoot at the hub
   * 
   * @return a new command to shoot at the hub
   */
  public Command shootAtHub() {
    return new ShootAtTargetCommand(
        indexerSubsystem,
        feederSubsystem,
        shooterSubsystem,
        drivetrainSubsystem,
        intakeSubsystem,
        ledSubsystem,
        () -> drivetrainSubsystem.getState().Pose,
        t -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? HUB_BLUE : HUB_RED);
  }

  public Command demoToss() {
    return new ShootCommand(
        indexerSubsystem,
        feederSubsystem,
        shooterSubsystem,
        drivetrainSubsystem,
        intakeSubsystem,
        ledSubsystem,
        Meters.of(0.5));
  }

  /**
   * Creates a command to shuttle fuel from any location to the nearest alliance-specific corner.
   * 
   * @return a new command to shuttle fuel to the corner
   */
  public Command shuttleToCorner() {
    return new ShootAtTargetCommand(
        indexerSubsystem,
        feederSubsystem,
        shooterSubsystem,
        drivetrainSubsystem,
        intakeSubsystem,
        ledSubsystem,
        () -> drivetrainSubsystem.getState().Pose,
        targetTranslation -> {
          Translation2d target;
          if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            target = targetTranslation.getY() > FIELD_WIDTH.in(Meters) / 2.0 ? SHUTTLE_BLUE_HIGH : SHUTTLE_BLUE_LOW;
          } else {
            target = targetTranslation.getY() > FIELD_WIDTH.in(Meters) / 2.0 ? SHUTTLE_RED_HIGH : SHUTTLE_RED_LOW;
          }
          // Adjust the target to be "offset distance" short of target along the vector between the robot and the target
          Translation2d vectorToTarget = target.minus(targetTranslation);
          double distanceToTarget = vectorToTarget.getNorm();
          Translation2d adjustedTarget = targetTranslation
              .plus(vectorToTarget.times((distanceToTarget - SHUTTLE_OFFSET_DISTANCE.in(Meters)) / distanceToTarget));
          return adjustedTarget;
        });
  }

  /**
   * Creates a new Command to control the drivetrain field-oriented using the provided translation and rotation
   * suppliers.
   *
   * @param translationXSupplier supplier for the robot's x translation velocity
   * @param translationYSupplier supplier for the robot's y translation velocity
   * @param omegaSupplier supplier for the robot's rotational velocity
   * @return a new Command to control the drivetrain
   */
  public Command drive(
      Supplier<LinearVelocity> translationXSupplier,
      Supplier<LinearVelocity> translationYSupplier,
      Supplier<AngularVelocity> omegaSupplier) {
    /* Setting up bindings for necessary control of the swerve drive platform */
    final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withCenterOfRotation(new Translation2d(Inches.of(6.0), Inches.zero())) // rotate around center with intake out
        .withDeadband(MAX_TELEOP_VELOCITY.times(0.01))
        .withRotationalDeadband(MAX_TELEOP_ANGULAR_VELOCITY.times(0.01))
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    return drivetrainSubsystem.applyRequest(
        () -> drive.withVelocityX(translationXSupplier.get())
            .withVelocityY(translationYSupplier.get())
            .withRotationalRate(omegaSupplier.get()));
  }

}
