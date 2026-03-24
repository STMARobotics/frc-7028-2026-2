package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj.LEDPattern.GradientType.kContinuous;
import static edu.wpi.first.wpilibj.util.Color.kBlack;
import static edu.wpi.first.wpilibj.util.Color.kGreen;
import static frc.robot.Constants.FieldConstants.FIELD_WIDTH;
import static frc.robot.Constants.ShootingConstants.HUB_SETPOINTS_BY_DISTANCE_METERS;
import static frc.robot.Constants.ShootingConstants.SHUTTLE_BLUE_HIGH;
import static frc.robot.Constants.ShootingConstants.SHUTTLE_BLUE_LOW;
import static frc.robot.Constants.ShootingConstants.SHUTTLE_OFFSET_DISTANCE;
import static frc.robot.Constants.ShootingConstants.SHUTTLE_RED_HIGH;
import static frc.robot.Constants.ShootingConstants.SHUTTLE_RED_LOW;
import static frc.robot.Constants.ShootingConstants.SHUTTLE_SETPOINTS_BY_DISTANCE_METERS;
import static frc.robot.Constants.ShootingConstants.TARGET_BLUE;
import static frc.robot.Constants.ShootingConstants.TARGET_RED;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_ANGULAR_VELOCITY;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_VELOCITY;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.commands.ShootAtTargetCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.LEDSubsystemContainer.IntakeLEDSubsystem;
import frc.robot.subsystems.LEDSubsystemContainer.RobotLEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import java.util.function.Supplier;

/**
 * Factory class for creating commands with the necessary subsystem dependencies.
 */
public class CommandFactory {
  private final CommandSwerveDrivetrain drivetrainSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final SpindexerSubsystem spindexerSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final IntakeSubsytem intakeSubsystem;
  private final IntakeLEDSubsystem intakeLEDSubsystem;
  private final RobotLEDSubsystem robotLEDSubsystem;

  /**
   * Constructor for Commandfactory, takes in all subsystems as parameters to use in command creation methods.
   * 
   * @param drivetrainSubsystem drivetrain subsystem
   * @param shooterSubsystem shooter subsystem
   * @param spindexerSubsystem spindexer subsystem
   * @param feederSubsystem feeder subsystem
   * @param intakeSubsystem intake subsystem
   * @param ledSubsystem led subsystem
   */
  public CommandFactory(
      CommandSwerveDrivetrain drivetrainSubsystem,
      ShooterSubsystem shooterSubsystem,
      SpindexerSubsystem spindexerSubsystem,
      FeederSubsystem feederSubsystem,
      IntakeSubsytem intakeSubsystem,
      IntakeLEDSubsystem intakeLEDSubsystem,
      RobotLEDSubsystem robotLEDSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.spindexerSubsystem = spindexerSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.intakeLEDSubsystem = intakeLEDSubsystem;
    this.robotLEDSubsystem = robotLEDSubsystem;
  }

  /**
   * Creates a command to shoot at the hub
   * 
   * @return a new command to shoot at the hub
   */
  public Command shootAtHub() {
    return new ShootAtTargetCommand(
        shooterSubsystem,
        feederSubsystem,
        spindexerSubsystem,
        robotLEDSubsystem,
        () -> drivetrainSubsystem.getState().Pose,
        drivetrainSubsystem::getCurrentFieldChassisSpeeds,
        t -> DriverStation.getAlliance().orElse(Blue) == Blue ? TARGET_BLUE : TARGET_RED,
        HUB_SETPOINTS_BY_DISTANCE_METERS);
  }

  /**
   * Creates a command to shuttle fuel from any location to the nearest alliance-specific corner.
   * 
   * @return a new command to shuttle fuel to the corner
   */
  public Command shuttleToCorner() {
    return new ShootAtTargetCommand(
        shooterSubsystem,
        feederSubsystem,
        spindexerSubsystem,
        robotLEDSubsystem,
        () -> drivetrainSubsystem.getState().Pose,
        drivetrainSubsystem::getCurrentFieldChassisSpeeds,
        shooterTranslation -> {
          Translation2d target;
          if (DriverStation.getAlliance().orElse(Blue) == Blue) {
            target = shooterTranslation.getY() > FIELD_WIDTH.in(Meters) / 2.0 ? SHUTTLE_BLUE_HIGH : SHUTTLE_BLUE_LOW;
          } else {
            target = shooterTranslation.getY() > FIELD_WIDTH.in(Meters) / 2.0 ? SHUTTLE_RED_HIGH : SHUTTLE_RED_LOW;
          }
          // Adjust the target to be "offset distance" short of target along the vector between the robot and the target
          Translation2d vectorToTarget = target.minus(shooterTranslation);
          double distanceToTarget = vectorToTarget.getNorm();
          Translation2d adjustedTarget = shooterTranslation
              .plus(vectorToTarget.times((distanceToTarget - SHUTTLE_OFFSET_DISTANCE.in(Meters)) / distanceToTarget));
          return adjustedTarget;
        },
        SHUTTLE_SETPOINTS_BY_DISTANCE_METERS);
  }

  /**
   * Creates a command that retracts and deploys the intake repeatedly to agitate the fuel.
   * 
   * @return a new command to agitate the intake
   */
  public Command agitateIntakeCommand() {
    return new RetractIntakeCommand(intakeSubsystem).until(intakeSubsystem::isRetracted)
        .andThen(new DeployIntakeCommand(intakeSubsystem))
        .alongWith(Commands.run(intakeSubsystem::runIntake))
        .alongWith(
            intakeLEDSubsystem.runPatternOnAllCommand(
                LEDPattern.gradient(kContinuous, kGreen, kBlack).scrollAtRelativeSpeed(Percent.per(Second).of(200))));
  }

  /**
   * Creates a command to shoot at the hub and also drive the robot using the passed translation and rotation suppliers.
   * 
   * @param translationXSupplier supplier for the robot's x translation velocity
   * @param translationYSupplier supplier for the robot's y translation velocity
   * @param omegaSupplier supplier for the robot's rotational velocity
   * @return a new ShootAtTargetCommand that also drives the robot
   */
  public Command shootAtHubWhileDriving(
      Supplier<LinearVelocity> translationXSupplier,
      Supplier<LinearVelocity> translationYSupplier,
      Supplier<AngularVelocity> omegaSupplier) {
    return shootAtHub().alongWith(drive(translationXSupplier, translationYSupplier, omegaSupplier));
  }

  /**
   * Creates a new Command to control the drivetrain using the provided translation and rotation suppliers.
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
