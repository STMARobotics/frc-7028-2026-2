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

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TuneShootingCommand;
import frc.robot.commands.led.DefaultLEDCommand;
import frc.robot.commands.led.LEDBootAnimationCommand;
import frc.robot.controls.ControlBindings;
import frc.robot.controls.DemoJoystickBindings;
import frc.robot.controls.JoystickControlBindings;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.stream.Stream;

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
      TunerConstants.FrontLeft,
      TunerConstants.FrontRight,
      TunerConstants.BackLeft,
      TunerConstants.BackRight);

  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  @Logged
  private final IntakeSubsytem intakeSubsystem = new IntakeSubsytem();
  @Logged
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final boolean demoMode = true;

  private final CommandFactory commandFactory = new CommandFactory(
      drivetrain,
      shooterSubsystem,
      indexerSubsystem,
      feederSubsystem,
      intakeSubsystem,
      ledSubsystem);

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  private final ControlBindings controlBindings;

  public RobotContainer() {
    // Configure control binding scheme
    if (demoMode) {
      controlBindings = new DemoJoystickBindings();
    } else if (DriverStation.getJoystickIsXbox(0) || Robot.isSimulation()) {
      controlBindings = new XBoxControlBindings();
    } else {
      controlBindings = new JoystickControlBindings();
    }

    // Configure and populate the auto command chooser with autos from PathPlanner
    configureAutoBuilder();
    configurePathPlannerCommands();
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(stream -> stream.flatMap(auto -> {
      if (auto.getName().startsWith("Left")) {
        var mirrored = new PathPlannerAuto(auto.getName(), true);
        mirrored.setName(auto.getName().replaceFirst("Left", "Right"));
        return Stream.of(auto, mirrored);
      } else {
        return Stream.of(auto);
      }
    }));
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();

    new LocalizationSubsystem(drivetrain::addVisionMeasurement, drivetrain::getIMUYawVelocity);

    // Warmup PathPlanner to avoid Java pauses
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

    // Run the boot animation
    var bootAnimation = new LEDBootAnimationCommand(ledSubsystem);
    CommandScheduler.getInstance().schedule(bootAnimation);

    // Set up default commmands
    ledSubsystem.setDefaultCommand(new DefaultLEDCommand(ledSubsystem));
    intakeSubsystem.setDefaultCommand(new IntakeCommand(intakeSubsystem, ledSubsystem));
  }

  private void configureBindings() {
    // Default drivetrain command for teleop control
    drivetrain.setDefaultCommand(
        commandFactory.drive(controlBindings.translationX(), controlBindings.translationY(), controlBindings.omega()));

    controlBindings.wheelsToX().ifPresent(trigger -> trigger.whileTrue(drivetrain.applyRequest(() -> brake)));
    controlBindings.resetFieldPosition().ifPresent(trigger -> trigger.onTrue(Commands.runOnce(() -> {
      Pose3d newPose = DriverStation.getAlliance().orElse(Blue) == Blue ? RESET_POSE_BLUE : RESET_POSE_RED;
      drivetrain.resetPose(newPose.toPose2d());
    })));

    // Intake controls
    controlBindings.runIntake().ifPresent(trigger -> trigger.onTrue(new IntakeCommand(intakeSubsystem, ledSubsystem)));

    controlBindings.stopIntake().ifPresent(trigger -> trigger.onTrue(intakeSubsystem.run(intakeSubsystem::stop)));

    controlBindings.eject().ifPresent(trigger -> trigger.whileTrue(Commands.run(() -> {
      intakeSubsystem.reverseIntake();
      ledSubsystem.runPattern(LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of(200)));
    }, intakeSubsystem, indexerSubsystem, ledSubsystem).finallyDo(() -> {
      intakeSubsystem.stop();
      indexerSubsystem.stop();
      ledSubsystem.off();
    })));

    controlBindings.deployIntake().ifPresent(trigger -> trigger.onTrue(new DeployIntakeCommand(intakeSubsystem)));

    controlBindings.retractIntake().ifPresent(trigger -> trigger.onTrue(new RetractIntakeCommand(intakeSubsystem)));

    // Shooting controls
    controlBindings.manualShoot()
        .ifPresent(
            trigger -> trigger.whileTrue(
                new ShootCommand(
                    indexerSubsystem,
                    feederSubsystem,
                    shooterSubsystem,
                    drivetrain,
                    intakeSubsystem,
                    ledSubsystem,
                    Meters.of(3.335))));

    controlBindings.tuneShoot()
        .ifPresent(
            trigger -> trigger.whileTrue(
                new TuneShootingCommand(
                    indexerSubsystem,
                    feederSubsystem,
                    shooterSubsystem,
                    ledSubsystem,
                    intakeSubsystem,
                    () -> drivetrain.getState().Pose)));

    controlBindings.autoShoot().ifPresent(trigger -> trigger.whileTrue(commandFactory.shootAtHub()));

    controlBindings.shuttle().ifPresent(trigger -> trigger.whileTrue(commandFactory.shuttleToCorner()));

    controlBindings.demoToss().ifPresent(trigger -> trigger.whileTrue(commandFactory.demoToss()));

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
            drivetrain::resetPose, // Consumer for seeding pose against auto
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
    NamedCommands.registerCommand("Intake", new IntakeCommand(intakeSubsystem, ledSubsystem));
    NamedCommands.registerCommand(
        "RetractIntake",
          new RetractIntakeCommand(intakeSubsystem).alongWith(new DefaultLEDCommand(ledSubsystem)));
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

    // Indexer
    SmartDashboard.putData("Indexer Quasi Fwd", indexerSubsystem.sysIdIndexerQuasistaticCommand(kForward));
    SmartDashboard.putData("Indexer Quasi Rev", indexerSubsystem.sysIdIndexerQuasistaticCommand(kReverse));
    SmartDashboard.putData("Indexer Dynam Fwd", indexerSubsystem.sysIdIndexerDynamicCommand(kForward));
    SmartDashboard.putData("Indexer Dynam Rev", indexerSubsystem.sysIdIndexerDynamicCommand(kReverse));

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