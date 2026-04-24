package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.FieldConstants.FIELD_WIDTH;
import static frc.robot.Constants.IntakeConstants.RETRACTED_POSITION;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.generated.TunerConstants;

public final class Constants {

  private Constants() {
  } // prevent instantiation

  public static final CANBus CANIVORE_BUS = new CANBus("canivore");
  // Robot dimensions INCLUDING bumpers
  public static final Distance ROBOT_WIDTH = Meters.of(0.932);
  public static final Distance ROBOT_LENGTH = Meters.of(0.776288);

  /**
   * Constants for the field dimensions of the WELDED field.
   */
  public static class FieldConstants {
    public static final Distance FIELD_LENGTH = Inches.of(651.2);
    public static final Distance FIELD_WIDTH = Inches.of(317.7);

    /**
     * Checks if a translation is within the field boundaries
     * 
     * @param translation The translation to check
     * @return true if the translation is within the field boundaries
     */
    public static boolean isValidFieldTranslation(Translation3d translation) {
      return isValidFieldTranslation(translation.toTranslation2d());
    }

    /**
     * Checks if a translation is within the field boundaries
     * 
     * @param translation The translation to check
     * @return true if the translation is within the field boundaries
     */
    public static boolean isValidFieldTranslation(Translation2d translation) {
      return translation.getX() >= 0.0 && translation.getX() <= FIELD_LENGTH.in(Meters) && translation.getY() >= 0.0
          && translation.getY() <= FIELD_WIDTH.in(Meters);
    }
  }

  /**
   * Constants for teleoperated driver control
   */
  public static class TeleopDriveConstants {
    /** Max velocity the driver can request */
    public static final LinearVelocity MAX_TELEOP_VELOCITY = TunerConstants.kSpeedAt12Volts;
    public static final LinearVelocity MAX_DEMO_VELOCITY = TunerConstants.kSpeedAt12Volts.times(0.25);
    /** Max angular velocity the driver can request */
    public static final AngularVelocity MAX_TELEOP_ANGULAR_VELOCITY = RotationsPerSecond.of(1.75);
    public static final AngularVelocity MAX_DEMO_ANGULAR_VELOCITY = RotationsPerSecond.of(0.5);

    /** Multiplier for shooting in teleop to reduce driver speed while shooting */
    public static final double SHOOT_VELOCITY_MULTIPLIER = 0.325;
    /** Blue reset pose is the blue corner, bumpers against the walls, facing downfield. */
    public static final Pose3d RESET_POSE_BLUE = new Pose3d(
        new Translation3d(ROBOT_LENGTH.in(Meters) / 2.0, ROBOT_WIDTH.in(Meters) / 2.0, 0.0),
        new Rotation3d(0.0, 0.0, Math.PI));
    /** Rotate around center with intake out */
    public static final Translation2d CENTER_OF_ROTATION = new Translation2d(Inches.of(6.0), Inches.zero());
    /** Red reset pose is the red corner, bumpers against the walls, facing downfield. */
    public static final Pose3d RESET_POSE_RED = new Pose3d(FlippingUtil.flipFieldPose(RESET_POSE_BLUE.toPose2d()));
  }

  /**
   * Constants for the shooter subsystem
   */
  public static class ShooterConstants {
    public static final int DEVICE_ID_FLYWHEEL_LEADER = 26; // Right side
    public static final int DEVICE_ID_FLYWHEEL_FOLLOWER_0 = 27; // Right side
    public static final int DEVICE_ID_FLYWHEEL_FOLLOWER_1 = 28; // Left side
    public static final int DEVICE_ID_FLYWHEEL_FOLLOWER_2 = 29; // Left side

    public static final Current FLYWHEEL_PEAK_TORQUE_CURRENT_FORWARD = Amps.of(160);
    // Reverse current is positive to allow for increased P for rapid recovery, while avoiding negative output when
    // there is no load. A tradeoff is that this will increase the time it takes to adjust the flywheel speed downward.
    public static final Current FLYWHEEL_PEAK_TORQUE_CURRENT_REVERSE = Amps.of(15);
    public static final Current FLYWHEEL_STATOR_CURRENT_LIMIT = Amps.of(170);
    public static final Current FLYWHEEL_SUPPLY_CURRENT_LIMIT = Amps.of(80);

    public static final SlotConfigs FLYWHEEL_SLOT_CONFIGS = new SlotConfigs().withKP(23.0).withKS(3.0);

    // The robot shoots out the back
    public static final Rotation2d SHOOTER_OFFSET_ANGLE = Rotation2d.kPi;

    public static final AngularVelocity FLYWHEEL_VELOCITY_TOLERANCE = RotationsPerSecond.of(1.0);
    public static final AngularVelocity FLYWHEEL_EJECT_VELOCITY = RotationsPerSecond.of(-15);

    public static final Angle FUEL_EXIT_ANGLE = Degrees.of(70);
  }

  /**
   * Constants for vision processing
   */
  public static class VisionConstants {
    public static final int DEVICE_ID_MITOCANDRIA = 0;
    public static final String[] APRILTAG_CAMERA_NAMES = { "limelight-left", "limelight-right", "limelight-back" };
    public static final Transform3d[] ROBOT_TO_CAMERA_TRANSFORMS = new Transform3d[] {
        new Transform3d(
            new Translation3d(Inches.of(-6.230084), Inches.of(-14.513163), Inches.of(15.352343)),
            new Rotation3d(0.0, degreesToRadians(16.0), Math.PI / 2.0)),
        new Transform3d(
            new Translation3d(Inches.of(-6.230084), Inches.of(14.513163), Inches.of(15.352343)),
            new Rotation3d(0.0, degreesToRadians(16.0), -Math.PI / 2.0)),
        new Transform3d(
            new Translation3d(Inches.of(-11.1515), Inches.of(6.75), Inches.of(10.012)),
            new Rotation3d(0.0, degreesToRadians(26.0), Math.PI)) };

    public static final int LIMELIGHT_BLUE_PIPELINE = 0;
    public static final int LIMELIGHT_RED_PIPELINE = 1;

    // The standard deviations of our vision estimated poses, which affect correction rate
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(2.0, 2.0, 8.0);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1.0);
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.3;

    /** The max average distance for AprilTag measurements to be considered valid */
    public static final Distance TAG_DISTANCE_THRESHOLD = Meters.of(3.5);

    /** The max distance from the starting pose for AprilTag measurements to be considered valid */
    public static final Distance STARTING_DISTANCE_THRESHOLD = Meters.of(3.0);

    /** The robot angular velocity threshold for accepting vision measurements */
    public static final AngularVelocity ANGULAR_VELOCITY_THRESHOLD = DegreesPerSecond.of(720);

    /**
     * The threshold for the error between the best AprilTag pose estimate and the QuestNav pose measurements for the
     * QuestNav pose to be considered valid
     */
    public static final Distance QUESTNAV_APRILTAG_ERROR_THRESHOLD = Meters.of(0.5);
  }

  public static class IntakeConstants {
    // Device IDs
    public static final int DEVICE_ID_DEPLOY_MOTOR = 10;
    public static final int DEVICE_ID_ROLLER_MOTOR = 11; // left
    public static final int DEVICE_ID_ROLLER_FOLLOWER = 12; // right
    public static final int CHANNEL_ID_DEPLOY_POTENTIOMETER = 3;

    // Roller constants
    public static final Current ROLLER_PEAK_TORQUE_CURRENT_FORWARD = Amps.of(170);
    public static final Current ROLLER_PEAK_TORQUE_CURRENT_REVERSE = ROLLER_PEAK_TORQUE_CURRENT_FORWARD.unaryMinus();
    public static final Current ROLLER_STATOR_CURRENT_LIMIT = Amps.of(190);
    public static final Current ROLLER_SUPPLY_CURRENT_LIMIT = Amps.of(80);
    public static final SlotConfigs ROLLER_SLOT_CONFIGS = new SlotConfigs().withKP(12).withKS(5.1);

    public static final AngularVelocity ROLLER_INTAKE_VELOCITY = RotationsPerSecond.of(80.0);
    public static final AngularVelocity ROLLER_INTAKE_SHOOTING_VELOCITY = RotationsPerSecond.of(40.0);
    public static final AngularVelocity ROLLER_EJECT_VELOCITY = RotationsPerSecond.of(-30.0);

    // Deploy constants
    public static final Current DEPLOY_STATOR_CURRENT_LIMIT = Amps.of(40);
    public static final Current DEPLOY_SUPPLY_CURRENT_LIMIT = Amps.of(30);
    public static final Current DEPLOY_PEAK_CURRENT_FORWARD = Amps.of(50);
    public static final Current DEPLOY_PEAK_CURRENT_REVERSE = DEPLOY_PEAK_CURRENT_FORWARD.unaryMinus();

    // Deploy limits in motor angle
    public static final Angle DEPLOY_REVERSE_LIMIT = Rotations.of(0); // Retracted
    public static final Angle DEPLOY_FORWARD_LIMIT = Rotations.of(18.38); // Deployed
    // Deploy limits in potentiometer values
    public static final double POTENTIOMETER_REVERSE_LIMIT = 0.568;
    public static final double POTENTIOMETER_FORWARD_LIMIT = 0.939;
    // Calculated "full range" of the potentiometer in motor rotations, if it was capable of turning all 10 turns
    public static final Angle POTENTIOMETER_FULL_RANGE = DEPLOY_FORWARD_LIMIT.minus(DEPLOY_REVERSE_LIMIT)
        .div(POTENTIOMETER_FORWARD_LIMIT - POTENTIOMETER_REVERSE_LIMIT);
    // The offset of the potentiometer from the actual position of the intake in motor angle
    public static final Angle POTENTIOMETER_OFFSET = Rotations.of(-28.13);

    public static final SlotConfigs DEPLOY_SLOT_CONFIGS = new SlotConfigs().withGravityType(GravityTypeValue.Arm_Cosine)
        .withKP(5.0)
        .withKS(0.0)
        .withKV(0.0);
    public static final MotionMagicConfigs DEPLOY_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(120.0)
        .withMotionMagicCruiseVelocity(180.0);

    public static final Angle DEPLOYED_POSITION = DEPLOY_FORWARD_LIMIT.minus(Rotations.of(0.25));
    public static final Angle RETRACTED_POSITION = DEPLOY_REVERSE_LIMIT.plus(Rotations.of(0.25));

    public static final Pose3d POSE_DEPLOYED = new Pose3d(0.267, 0.0, -0.043, Rotation3d.kZero);
    public static final Pose3d POSE_RETRACTED = new Pose3d(0.0, 0.0, 0.0, Rotation3d.kZero);

    public static final Angle DEPLOY_TOLERANCE = Rotations.of(0.2);
    public static final Current DEPLOY_SHOOTING_CURRENT = Amps.of(-17.0);
  }

  public static class IndexerConstants {
    public static final int DEVICE_ID_INDEXER_MOTOR_LEADER = 15;
    public static final int DEVICE_ID_INDEXER_MOTOR_FOLLOWER = 16;

    public static final Current INDEXER_PEAK_TORQUE_CURRENT_FORWARD = Amps.of(80);
    public static final Current INDEXER_PEAK_TORQUE_CURRENT_REVERSE = INDEXER_PEAK_TORQUE_CURRENT_FORWARD.unaryMinus();
    public static final Current INDEXER_STATOR_CURRENT_LIMIT = Amps.of(80);
    public static final Current INDEXER_SUPPLY_CURRENT_LIMIT = Amps.of(50);

    public static final SlotConfigs INDEXER_SLOT_CONFIGS = new SlotConfigs().withKP(0.5).withKV(0.02).withKS(4.0);

    public static final AngularVelocity INDEXER_FEED_VELOCITY = RotationsPerSecond.of(95);
    public static final AngularVelocity INDEXER_EJECT_VELOCITY = RotationsPerSecond.of(-80);

  }

  /**
   * Constants for the Feeder Subsystem
   */
  public static class FeederConstants {
    public static final int DEVICE_ID_FEEDER_LEADER = 20;
    public static final int DEVICE_ID_FEEDER_FOLLOWER = 21;

    public static final Current FEEDER_PEAK_TORQUE_CURRENT_FORWARD = Amps.of(100);
    public static final Current FEEDER_PEAK_TORQUE_CURRENT_REVERSE = FEEDER_PEAK_TORQUE_CURRENT_FORWARD.unaryMinus();
    public static final Current FEEDER_STATOR_CURRENT_LIMIT = Amps.of(110);
    public static final Current FEEDER_SUPPLY_CURRENT_LIMIT = Amps.of(80);
    public static final SlotConfigs FEEDER_SLOT_CONFIGS = new SlotConfigs().withKP(3).withKS(3).withKV(0.05);

    public static final AngularVelocity FEEDER_FEED_VELOCITY = RotationsPerSecond.of(95);
    public static final AngularVelocity FEEDER_EJECT_VELOCITY = RotationsPerSecond.of(-25);
  }

  /**
   * Constants for the LEDs
   */
  public static class LEDConstants {
    public static final int DEVICE_ID_LEDS = 9;

    public static final int LED_STRIP_LENGTH = 36;
    public static final int STRIP_COUNT = 2;

    public static final int TOTAL_LEDS = LED_STRIP_LENGTH * STRIP_COUNT;
  }

  /**
   * Constants related to shooting fuel
   * <p>
   * These constants are not specific to the shooter subsystem, they are about the process of shooting.
   */
  public static class ShootingConstants {
    public static final Angle AIM_TOLERANCE = Degrees.of(1.5);
    public static final double HEADING_P = 6.0;

    // Parameters for the intake shooting sequence
    public static final Time RETRACT_INTAKE_DELAY = Seconds.of(0.0);
    public static final Time JAM_DEBOUNCE_TIME = Seconds.of(0.2);
    public static final AngularVelocity JAM_THRESHOLD = RotationsPerSecond.of(-3.0);
    public static final Time UNJAM_DURATION = Seconds.of(0.25);
    public static final Angle RETRACTED_THRESHOLD = RETRACTED_POSITION;

    private static InterpolatingDoubleTreeMap createShooterInterpolator() {
      var map = new InterpolatingDoubleTreeMap();
      map.put(2.099, 27.0);
      map.put(5.302, 37.0);

      return map;
    }

    public static final InterpolatingDoubleTreeMap HUB_SETPOINTS_BY_DISTANCE_METERS = createShooterInterpolator();

    private static InterpolatingDoubleTreeMap createShuttleInterpolator() {
      var map = new InterpolatingDoubleTreeMap();
      map.put(2.0, 20.0);
      map.put(6.0, 40.0);
      return map;
    }

    public static final InterpolatingDoubleTreeMap SHUTTLE_SETPOINTS_BY_DISTANCE_METERS = createShuttleInterpolator();

    /** Translation of the hub on the blue side */
    public static final Translation2d HUB_BLUE = new Translation2d(Inches.of(182.143595), Inches.of(158.84375));

    /** Translation of the hub on the red side */
    public static final Translation2d HUB_RED = new Translation2d(Inches.of(469.078905), Inches.of(158.84375));

    /** Translations for shuttling on the blue side, with Z > 1/2 of the field */
    public static final Translation2d SHUTTLE_BLUE_HIGH = new Translation2d(
        Inches.of(2.0),
        FIELD_WIDTH.minus(Inches.of(42)));
    /** Translations for shuttling on the blue side, with Z < 1/2 of the field */
    public static final Translation2d SHUTTLE_BLUE_LOW = new Translation2d(Inches.of(2.0), Inches.of(42));

    /** Translations for shuttling on the red side, with Z > 1/2 of the field */
    public static final Translation2d SHUTTLE_RED_HIGH = FlippingUtil.flipFieldPosition(SHUTTLE_BLUE_LOW);
    /** Translations for shuttling on the red side, with Z < 1/2 of the field */
    public static final Translation2d SHUTTLE_RED_LOW = FlippingUtil.flipFieldPosition(SHUTTLE_BLUE_HIGH);

    /**
     * The offset distance from the shuttle target where the robot will shoot. It will shoot offset distance short
     * (closer to the robot) of the target
     */
    public static final Distance SHUTTLE_OFFSET_DISTANCE = Meters.of(3.125);
  }
}
