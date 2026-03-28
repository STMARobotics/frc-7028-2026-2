package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.FieldConstants.FIELD_LENGTH;
import static frc.robot.Constants.FieldConstants.FIELD_WIDTH;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
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
    public static final LinearVelocity MAX_TELEOP_VELOCITY = TunerConstants.kSpeedAt12Volts.times(0.7);
    /** Max angular velocity the driver can request */
    public static final AngularVelocity MAX_TELEOP_ANGULAR_VELOCITY = RotationsPerSecond.of(1.75);
    /** Multiplier for shooting in teleop to reduce driver speed while shooting */
    public static final double SHOOT_VELOCITY_MULTIPLIER = 0.325;
    /** Blue reset pose is the blue corner, bumpers against the walls, facing downfield. */
    public static final Pose3d RESET_POSE_BLUE = new Pose3d(
        new Translation3d(ROBOT_LENGTH.in(Meters) / 2.0, ROBOT_WIDTH.in(Meters) / 2.0, 0.0),
        new Rotation3d(0.0, 0.0, 0.0));
    /** Red reset pose is the red corner, bumpers against the walls, facing downfield. */
    public static final Pose3d RESET_POSE_RED = new Pose3d(FlippingUtil.flipFieldPose(RESET_POSE_BLUE.toPose2d()));
  }

  /**
   * Constants for odometry state estimation
   */
  public static class OdometryConstants {
    // Trust the physics/encoders moderately
    public static final Matrix<N3, N1> STATE_STD_DEVS = VecBuilder.fill(
        0.1, // X: 10cm error per meter (Trust wheels moderately)
          0.1, // Y: 10cm error per meter
          0.05 // Theta: 0.05 radians to (Trust the Pigeon heavily)
    );
  }

  /**
   * Constants for the shooter subsystem
   */
  public static class ShooterConstants {
    public static final int DEVICE_ID_FLYWHEEL_LEADER = 26;
    public static final int DEVICE_ID_FLYWHEEL_FOLLOWER_0 = 27;
    public static final int DEVICE_ID_FLYWHEEL_FOLLOWER_1 = 28;
    public static final int DEVICE_ID_FLYWHEEL_FOLLOWER_2 = 29;

    public static final Current FLYWHEEL_PEAK_TORQUE_CURRENT_FORWARD = Amps.of(160);
    // Reverse current is positive to allow for increased P for rapid recovery, while avoiding negative output when
    // there is no load. A tradeoff is that this will increase the time it takes to adjust the flywheel speed downward.
    public static final Current FLYWHEEL_PEAK_TORQUE_CURRENT_REVERSE = Amps.of(15);
    public static final Current FLYWHEEL_STATOR_CURRENT_LIMIT = Amps.of(170);
    public static final Current FLYWHEEL_SUPPLY_CURRENT_LIMIT = Amps.of(80);

    public static final SlotConfigs FLYWHEEL_SLOT_CONFIGS = new SlotConfigs().withKP(23.0).withKS(20.0);

    public static final AngularVelocity FLYWHEEL_VELOCITY_TOLERANCE = RotationsPerSecond.of(1.5);
  }

  /**
   * Constants for vision processing
   */
  public static class VisionConstants {
    public static final int DEVICE_ID_MITOCANDRIA = 0;
    public static final String[] APRILTAG_CAMERA_NAMES = { "limelight-left", "limelight-back" };// "limelight-right",
                                                                                                // "limelight-left",
    // "limelight-back"
    // };
    public static final Transform3d[] ROBOT_TO_CAMERA_TRANSFORMS = new Transform3d[] {
        new Transform3d(
            new Translation3d(-0.241813, 0.349273, 0.203479),
            new Rotation3d(0.0, degreesToRadians(28), -Math.PI / 2.0)),
        // new Transform3d(
        // new Translation3d(Inches.of(-15.062), Inches.of(-8.718), Inches.of(11.96)),
        // new Rotation3d(Math.PI, degreesToRadians(30), Math.PI / 2.0)) };
        new Transform3d(
            new Translation3d(Inches.of(-10.050), Inches.of(-11.04), Inches.of(12.015)),
            new Rotation3d(Math.PI, degreesToRadians(28), Math.PI)) };

    public static final int LIMELIGHT_BLUE_PIPELINE = 0;
    public static final int LIMELIGHT_RED_PIPELINE = 1;

    // The standard deviations of our vision estimated poses, which affect correction rate
    public static final double APRILTAG_TRANSLATION_STD_DEV = 0.05;
    public static final Matrix<N3, N1> APRILTAG_STD_DEVS = VecBuilder
        .fill(APRILTAG_TRANSLATION_STD_DEV, APRILTAG_TRANSLATION_STD_DEV, Double.MAX_VALUE);

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
    public static final int DEVICE_ID_ROLLER_MOTOR = 11;
    public static final int DEVICE_ID_ROLLER_FOLLOWER = 12;
    public static final int DEVICE_ID_DEPLOY_POTENTIOMETER = 1;

    // Roller constants
    public static final Current ROLLER_PEAK_TORQUE_CURRENT_FORWARD = Amps.of(100);
    public static final Current ROLLER_PEAK_TORQUE_CURRENT_REVERSE = ROLLER_PEAK_TORQUE_CURRENT_FORWARD.unaryMinus();
    public static final Current ROLLER_STATOR_CURRENT_LIMIT = Amps.of(120);
    public static final Current ROLLER_SUPPLY_CURRENT_LIMIT = Amps.of(60);
    public static final SlotConfigs ROLLER_SLOT_CONFIGS = new SlotConfigs().withKP(12).withKS(5.1);

    public static final AngularVelocity ROLLER_INTAKE_VELOCITY = RotationsPerSecond.of(80.0);
    public static final AngularVelocity ROLLER_EJECT_VELOCITY = RotationsPerSecond.of(-30.0);

    // Deploy constants
    public static final Current DEPLOY_STATOR_CURRENT_LIMIT = Amps.of(60);
    public static final Current DEPLOY_SUPPLY_CURRENT_LIMIT = Amps.of(30);

    public static final Angle DEPLOY_REVERSE_LIMIT = Rotations.of(0.0);
    public static final Angle DEPLOY_FORWARD_LIMIT = Rotations.of(0.152);

    public static final SlotConfigs DEPLOY_SLOT_CONFIGS = new SlotConfigs().withGravityType(GravityTypeValue.Arm_Cosine)
        .withKP(25.0)
        .withKS(0.6)
        .withKV(0.0);
    public static final MotionMagicConfigs DEPLOY_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(10.0)
        .withMotionMagicCruiseVelocity(15.0);

    public static final Angle DEPLOYED_POSITION = DEPLOY_REVERSE_LIMIT;
    public static final Angle RETRACTED_POSITION = DEPLOY_FORWARD_LIMIT.minus(Degrees.of(2.0));
    public static final Angle DEPLOY_TOLERANCE = Rotations.of(0.03);
  }

  public static class IndexerConstants {
    public static final int DEVICE_ID_INDEXER_MOTOR = 15;

    public static final Current INDEXER_PEAK_TORQUE_CURRENT_FORWARD = Amps.of(80);
    public static final Current INDEXER_PEAK_TORQUE_CURRENT_REVERSE = INDEXER_PEAK_TORQUE_CURRENT_FORWARD.unaryMinus();
    public static final Current INDEXER_STATOR_CURRENT_LIMIT = Amps.of(80);
    public static final Current INDEXER_SUPPLY_CURRENT_LIMIT = Amps.of(50);

    public static final SlotConfigs INDEXER_SLOT_CONFIGS = new SlotConfigs().withKP(5.0).withKV(0.0).withKS(50.0);

    public static final AngularVelocity INDEXER_FEED_VELOCITY = RotationsPerSecond.of(20);
    public static final AngularVelocity INDEXER_AGITATE_FORWARD_VELOCITY = RotationsPerSecond.of(3);
    public static final AngularVelocity INDEXER_AGITATE_BACKWARD_VELOCITY = RotationsPerSecond.of(-3);

  }

  /**
   * Constants for the Feeder Subsystem
   */
  public static class FeederConstants {
    public static final int DEVICE_ID_FEEDER_LEADER = 20;
    public static final int DEVICE_ID_FEEDER_FOLLOWER = 21;
    public static final int DEVICE_ID_FEEDER_CANRANGE = 20;

    public static final Current FEEDER_PEAK_TORQUE_CURRENT_FORWARD = Amps.of(140);
    public static final Current FEEDER_PEAK_TORQUE_CURRENT_REVERSE = FEEDER_PEAK_TORQUE_CURRENT_FORWARD.unaryMinus();
    public static final Current FEEDER_STATOR_CURRENT_LIMIT = Amps.of(150);
    public static final Current FEEDER_SUPPLY_CURRENT_LIMIT = Amps.of(80);
    public static final SlotConfigs FEEDER_SLOT_CONFIGS = new SlotConfigs().withKP(5).withKS(40);

    public static final AngularVelocity FEEDER_FEED_VELOCITY = RotationsPerSecond.of(80);
    public static final AngularVelocity FEEDER_UNJAM_VELOCITY = RotationsPerSecond.of(-25);
  }

  /**
   * Constants for the LEDs
   */
  public static class LEDConstants {
    public static final int DEVICE_ID_LEDS = 9;

    public static final int INTAKE_HIGH_LED_STRIP_LENGTH = 39;
    public static final int INTAKE_LOW_LED_STRIP_LENGTH = 35;
    public static final int LEFT_LED_STRIP_LENGTH = 23;
    public static final int BACK_LED_STRIP_LENGTH = 38;

    public static final int TOTAL_LEDS = INTAKE_HIGH_LED_STRIP_LENGTH + INTAKE_LOW_LED_STRIP_LENGTH
        + LEFT_LED_STRIP_LENGTH + BACK_LED_STRIP_LENGTH;
  }

  /**
   * Constants related to shooting fuel
   * <p>
   * These constants are not specific to the shooter subsystem, they are about the process of shooting.
   */
  public static class ShootingConstants {
    public static final Angle AIM_TOLERANCE = Degrees.of(1.5);

    // Danger zone is the region on the X axis where we don't want to shoot or shuttle from because of the trench
    public static final Distance DANGER_ZONE_MIN_BLUE = Meters.of(4.25);
    public static final Distance DANGER_ZONE_MAX_BLUE = Meters.of(4.95);
    public static final Distance DANGER_ZONE_MIN_RED = FIELD_LENGTH.minus(DANGER_ZONE_MAX_BLUE);
    public static final Distance DANGER_ZONE_MAX_RED = FIELD_LENGTH.minus(DANGER_ZONE_MIN_BLUE);

    private static InterpolatingDoubleTreeMap createShooterInterpolator() {
      var map = new InterpolatingDoubleTreeMap();
      map.put(0.0, 0.0);
      return map;
    }

    public static final InterpolatingDoubleTreeMap HUB_SETPOINTS_BY_DISTANCE_METERS = createShooterInterpolator();

    private static InterpolatingDoubleTreeMap createShuttleInterpolator() {
      var map = new InterpolatingDoubleTreeMap();
      map.put(0.0, 0.0);
      return map;
    }

    public static final InterpolatingDoubleTreeMap SHUTTLE_SETPOINTS_BY_DISTANCE_METERS = createShuttleInterpolator();

    /** Translation of the hub on the blue side */
    public static final Translation2d TARGET_BLUE = new Translation2d(Inches.of(182.143595), Inches.of(158.84375));

    /** Translation of the hub on the red side */
    public static final Translation2d TARGET_RED = new Translation2d(Inches.of(469.078905), Inches.of(158.84375));

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
