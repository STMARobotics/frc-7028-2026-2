// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse;
import static frc.robot.Constants.TeleopDriveConstants.RESET_POSE_BLUE;
import static frc.robot.Constants.TeleopDriveConstants.RESET_POSE_RED;
import static frc.robot.Constants.TeleopDriveConstants.SHOOT_VELOCITY_MULTIPLIER;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OdometryConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TuneShootingCommand;
import frc.robot.commands.led.DefaultLEDCommand;
import frc.robot.commands.led.LEDBootAnimationCommand;
import frc.robot.controls.ControlBindings;
import frc.robot.controls.JoystickControlBindings;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.LEDSubsystemContainer;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.MitoCANdriaSubsytem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class RobotContainer {
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds ppRobotSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds();
  private final DrivetrainTelemetry drivetrainTelemetry = new DrivetrainTelemetry();

  // Create the drivetrain subsystem here instead of using TunerConstants.createDrivetrain() to set standard deviations
  // without editing generated TunerConstants file
  public final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(
      TunerConstants.DrivetrainConstants,
      0,
      OdometryConstants.STATE_STD_DEVS,
      VisionConstants.APRILTAG_STD_DEVS,
      TunerConstants.FrontLeft,
      TunerConstants.FrontRight,
      TunerConstants.BackLeft,
      TunerConstants.BackRight);
  @Logged
  private final LocalizationSubsystem localizationSubsystem = new LocalizationSubsystem(
      drivetrain::addVisionMeasurement,
      drivetrain::resetPose,
      () -> drivetrain.getState().Pose,
      drivetrain::getIMUYawVelocity);
  @Logged
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  @Logged
  private final SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem();
  @Logged
  private final IntakeSubsytem intakeSubsystem = new IntakeSubsytem();
  @Logged
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  @Logged
  private final MitoCANdriaSubsytem mitoCANdriaSubsytem = new MitoCANdriaSubsytem();
  private final LEDSubsystemContainer ledSubsystem = new LEDSubsystemContainer();

  private final CommandFactory commandFactory = new CommandFactory(
      drivetrain,
      shooterSubsystem,
      spindexerSubsystem,
      feederSubsystem,
      intakeSubsystem,
      ledSubsystem.getIntakeLEDSubsystem(),
      ledSubsystem.getRobotLEDSubsystem());

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  private final ControlBindings controlBindings;

  public RobotContainer() {
    // Configure control binding scheme
    if (DriverStation.getJoystickIsXbox(0) || Robot.isSimulation()) {
      controlBindings = new XBoxControlBindings();
    } else {
      controlBindings = new JoystickControlBindings();
    }

    // Configure and populate the auto command chooser with autos from PathPlanner
    configureAutoBuilder();
    configurePathPlannerCommands();
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        "Left Double Dip",
          stream -> stream.filter(item -> !item.getRequirements().isEmpty()));
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();

    // Warmup PathPlanner to avoid Java pauses
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

    // Run the boot animation
    var bootAnimation = new LEDBootAnimationCommand(ledSubsystem.getIntakeLEDSubsystem());
    CommandScheduler.getInstance().schedule(bootAnimation);

    // Set up default commmands
    ledSubsystem.getIntakeLEDSubsystem().setDefaultCommand(new DefaultLEDCommand(ledSubsystem.getIntakeLEDSubsystem()));
    ledSubsystem.getRobotLEDSubsystem().setDefaultCommand(new DefaultLEDCommand(ledSubsystem.getRobotLEDSubsystem()));
  }

  private void configureBindings() {
    // Default drivetrain command for teleop control
    drivetrain.setDefaultCommand(
        commandFactory.drive(controlBindings.translationX(), controlBindings.translationY(), controlBindings.omega()));

    controlBindings.wheelsToX().ifPresent(trigger -> trigger.whileTrue(drivetrain.applyRequest(() -> brake)));
    controlBindings.resetFieldPosition().ifPresent(trigger -> trigger.onTrue(Commands.runOnce(() -> {
      Pose3d newPose = DriverStation.getAlliance().orElse(Blue) == Blue ? RESET_POSE_BLUE : RESET_POSE_RED;
      localizationSubsystem.resetPose(newPose);
    })));
    controlBindings.resetFieldPositionFromAprilTags()
        .ifPresent(
            trigger -> trigger
                .whileTrue(Commands.run(localizationSubsystem::resetPoseFromAprilTags).ignoringDisable(true)));

    // Intake controls
    controlBindings.runIntake()
        .ifPresent(trigger -> trigger.onTrue(new IntakeCommand(intakeSubsystem, ledSubsystem.getIntakeLEDSubsystem())));

    controlBindings.stopIntake().ifPresent(trigger -> trigger.onTrue(Commands.runOnce(() -> {
      intakeSubsystem.stop();
      ledSubsystem.getIntakeLEDSubsystem().off();
    }, intakeSubsystem, ledSubsystem.getIntakeLEDSubsystem())));

    controlBindings.eject().ifPresent(trigger -> trigger.whileTrue(Commands.run(() -> {
      intakeSubsystem.reverseIntake();
      spindexerSubsystem.agitate();
      ledSubsystem.getIntakeLEDSubsystem()
          .runPatternOnIntakeHigh(LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of(200)));
      ledSubsystem.getIntakeLEDSubsystem()
          .runPatternOnIntakeLow(LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of(200)));
    }, intakeSubsystem, spindexerSubsystem, ledSubsystem.getIntakeLEDSubsystem()).finallyDo(() -> {
      intakeSubsystem.stop();
      spindexerSubsystem.stop();
      ledSubsystem.getIntakeLEDSubsystem().off();
    })));

    controlBindings.deployIntake().ifPresent(trigger -> trigger.onTrue(new DeployIntakeCommand(intakeSubsystem)));

    controlBindings.retractIntake().ifPresent(trigger -> trigger.onTrue(new RetractIntakeCommand(intakeSubsystem)));

    // Shooting controls
    controlBindings.manualShoot()
        .ifPresent(
            trigger -> trigger
                .whileTrue(new ShootCommand(spindexerSubsystem, feederSubsystem, shooterSubsystem, Meters.of(2.0))));

    controlBindings.tuneShoot()
        .ifPresent(
            trigger -> trigger.whileTrue(
                new TuneShootingCommand(
                    spindexerSubsystem,
                    feederSubsystem,
                    shooterSubsystem,
                    ledSubsystem,
                    () -> drivetrain.getState().Pose)));

    controlBindings.autoShoot()
        .ifPresent(
            trigger -> trigger.whileTrue(
                commandFactory.shootAtHubWhileDriving(
                    () -> controlBindings.translationX().get().times(SHOOT_VELOCITY_MULTIPLIER),
                      () -> controlBindings.translationY().get().times(SHOOT_VELOCITY_MULTIPLIER),
                      () -> controlBindings.omega()
                          .get()
                          .times(SHOOT_VELOCITY_MULTIPLIER)
                          .times(SHOOT_VELOCITY_MULTIPLIER))));

    controlBindings.shuttle().ifPresent(trigger -> trigger.whileTrue(commandFactory.shuttleToCorner()));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final SwerveRequest idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    drivetrain.registerTelemetry(drivetrainTelemetry::telemeterize);
  }

  private void configureAutoBuilder() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> drivetrain.getState().Pose, // Supplier of current robot pose
            localizationSubsystem::resetPose, // Consumer for seeding pose against auto
            () -> drivetrain.getState().Speeds, // Supplier of current robot speeds
            // Consumer of ChassisSpeeds and feedforwards to drive the robot
            (speeds, feedforwards) -> drivetrain.setControl(
                ppRobotSpeedsRequest.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
            new PPHolonomicDriveController(
                // PID constants for translation
                new PIDConstants(10, 0, 0),
                // PID constants for rotation
                new PIDConstants(6, 0, 0)),
            config,
            // Assume the path needs to be flipped for Red vs Blue, this is normally the case
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            drivetrain // Subsystem for requirements
      );
    } catch (Exception ex) {
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  private void configurePathPlannerCommands() {
    NamedCommands.registerCommand("Shoot", commandFactory.shootAtHub());
    NamedCommands.registerCommand("AgitateIntake", commandFactory.agitateIntakeCommand());
    NamedCommands.registerCommand(
        "Intake",
          new DeployIntakeCommand(intakeSubsystem)
              .andThen(new IntakeCommand(intakeSubsystem, ledSubsystem.getIntakeLEDSubsystem())));
    NamedCommands.registerCommand("RetractIntake", new RetractIntakeCommand(intakeSubsystem));
    NamedCommands.registerCommand("Shuttle", commandFactory.shuttleToCorner());
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }

  /** Populate the SysID dashboard controls with commands for system identification */
  public void populateTestModeDashboard() {
    // Drive
    SmartDashboard.putData("Drive Quasi Fwd", drivetrain.sysIdTranslationQuasiCommand(kForward));
    SmartDashboard.putData("Drive Quasi Rev", drivetrain.sysIdTranslationQuasiCommand(kReverse));
    SmartDashboard.putData("Drive Dynam Fwd", drivetrain.sysIdTranslationDynamCommand(kForward));
    SmartDashboard.putData("Drive Dynam Rev", drivetrain.sysIdTranslationDynamCommand(kReverse));

    // Drive TorqueFOC
    SmartDashboard.putData("Drive Torque Quasi Fwd", drivetrain.sysIdTranslationQuasiTorqueCommand(kForward));
    SmartDashboard.putData("Drive Torque Quasi Rev", drivetrain.sysIdTranslationQuasiTorqueCommand(kReverse));
    SmartDashboard.putData("Drive Torque Dynam Fwd", drivetrain.sysIdTranslationDynamTorqueCommand(kForward));
    SmartDashboard.putData("Drive Torque Dynam Rev", drivetrain.sysIdTranslationDynamTorqueCommand(kReverse));

    // Steer
    SmartDashboard.putData("Steer Quasi Fwd", drivetrain.sysIdSteerQuasiCommand(kForward));
    SmartDashboard.putData("Steer Quasi Rev", drivetrain.sysIdSteerQuasiCommand(kReverse));
    SmartDashboard.putData("Steer Dynam Fwd", drivetrain.sysIdSteerDynamCommand(kForward));
    SmartDashboard.putData("Steer Dynam Rev", drivetrain.sysIdSteerDynamCommand(kReverse));

    // Rotation
    SmartDashboard.putData("Rotate Quasi Fwd", drivetrain.sysIdRotationQuasiCommand(kForward));
    SmartDashboard.putData("Rotate Quasi Rev", drivetrain.sysIdRotationQuasiCommand(kReverse));
    SmartDashboard.putData("Rotate Dynam Fwd", drivetrain.sysIdRotationDynamCommand(kForward));
    SmartDashboard.putData("Rotate Dynam Rev", drivetrain.sysIdRotationDynamCommand(kReverse));

    // Spindexer
    SmartDashboard.putData("Spindexer Quasi Fwd", spindexerSubsystem.sysIdSpindexerQuasistaticCommand(kForward));
    SmartDashboard.putData("Spindexer Quasi Rev", spindexerSubsystem.sysIdSpindexerQuasistaticCommand(kReverse));
    SmartDashboard.putData("Spindexer Dynam Fwd", spindexerSubsystem.sysIdSpindexerDynamicCommand(kForward));
    SmartDashboard.putData("Spindexer Dynam Rev", spindexerSubsystem.sysIdSpindexerDynamicCommand(kReverse));

    // Feeder
    SmartDashboard.putData("Feeder Quasi Fwd", feederSubsystem.sysIdFeederQuasistaticCommand(kForward));
    SmartDashboard.putData("Feeder Quasi Rev", feederSubsystem.sysIdFeederQuasistaticCommand(kReverse));
    SmartDashboard.putData("Feeder Dynam Fwd", feederSubsystem.sysIdFeederDynamicCommand(kForward));
    SmartDashboard.putData("Feeder Dynam Rev", feederSubsystem.sysIdFeederDynamicCommand(kReverse));

    // Intake
    SmartDashboard.putData("Intake Deploy Quasi Fwd", intakeSubsystem.sysIdDeployQuasistaticCommand(kForward));
    SmartDashboard.putData("Intake Deploy Quasi Rev", intakeSubsystem.sysIdDeployQuasistaticCommand(kReverse));
    SmartDashboard.putData("Intake Deploy Dynam Fwd", intakeSubsystem.sysIdDeployDynamicCommand(kForward));
    SmartDashboard.putData("Intake Deploy Dynam Rev", intakeSubsystem.sysIdDeployDynamicCommand(kReverse));

    SmartDashboard.putData("Intake Roller Quasi Fwd", intakeSubsystem.sysIdRollerQuasistaticCommand(kForward));
    SmartDashboard.putData("Intake Roller Quasi Rev", intakeSubsystem.sysIdRollerQuasistaticCommand(kReverse));
    SmartDashboard.putData("Intake Roller Dynam Fwd", intakeSubsystem.sysIdRollerDynamicCommand(kForward));
    SmartDashboard.putData("Intake Roller Dynam Rev", intakeSubsystem.sysIdRollerDynamicCommand(kReverse));

    // Shooter
    SmartDashboard.putData("Shooter Flywheel Quasi Fwd", shooterSubsystem.sysIdFlywheelQuasistaticCommand(kForward));
    SmartDashboard.putData("Shooter Flywheel Quasi Rev", shooterSubsystem.sysIdFlywheelQuasistaticCommand(kReverse));
    SmartDashboard.putData("Shooter Flywheel Dynam Fwd", shooterSubsystem.sysIdFlywheelDynamicCommand(kForward));
    SmartDashboard.putData("Shooter Flywheel Dynam Rev", shooterSubsystem.sysIdFlywheelDynamicCommand(kReverse));
  }
}