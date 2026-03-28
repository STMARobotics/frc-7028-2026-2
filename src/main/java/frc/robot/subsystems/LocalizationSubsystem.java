package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static frc.robot.Constants.FieldConstants.isValidFieldTranslation;
import static frc.robot.Constants.VisionConstants.ANGULAR_VELOCITY_THRESHOLD;
import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_NAMES;
import static frc.robot.Constants.VisionConstants.APRILTAG_TRANSLATION_STD_DEV;
import static frc.robot.Constants.VisionConstants.LIMELIGHT_BLUE_PIPELINE;
import static frc.robot.Constants.VisionConstants.LIMELIGHT_RED_PIPELINE;
import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA_TRANSFORMS;
import static frc.robot.Constants.VisionConstants.TAG_DISTANCE_THRESHOLD;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.VisionMeasurementConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Subsystem for the localization system.
 * <p>
 * This subsystem integrates Limelight and QuestNav vision systems to provide pose estimation and field localization.
 * It manages camera pose configuration, vision measurement consumption, and field position validation.
 * Vision measurements are fused from multiple sources and published for use by other subsystems.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class LocalizationSubsystem extends SubsystemBase {

  private final VisionMeasurementConsumer visionMeasurementConsumer;
  private final Consumer<Pose2d> poseResetConsumer;
  private final Supplier<AngularVelocity> robotAngularVelocitySupplier;

  /**
   * Constructs a new LocalizationSubsystem.
   *
   * @param addVisionMeasurement the consumer for vision-based pose measurements
   * @param poseResetConsumer the consumer for resetting the robot's pose when the robot is disabled
   * @param poseSupplier supplier for the robot's current pose estimate from the fused estimator
   * @param angularVelocitySupplier supplier for the robot's current angular velocity, NOT from the fused estimator but
   *          directly from the IMU
   */
  public LocalizationSubsystem(
      VisionMeasurementConsumer addVisionMeasurement,
      Consumer<Pose2d> poseResetConsumer,
      Supplier<Pose2d> poseSupplier,
      Supplier<AngularVelocity> angularVelocitySupplier) {
    this.visionMeasurementConsumer = addVisionMeasurement;
    this.poseResetConsumer = poseResetConsumer;
    this.robotAngularVelocitySupplier = angularVelocitySupplier;

    for (int i = 0; i < APRILTAG_CAMERA_NAMES.length; i++) {
      LimelightHelpers.setCameraPose_RobotSpace(
          APRILTAG_CAMERA_NAMES[i],
            ROBOT_TO_CAMERA_TRANSFORMS[i].getX(),
            ROBOT_TO_CAMERA_TRANSFORMS[i].getY(),
            ROBOT_TO_CAMERA_TRANSFORMS[i].getZ(),
            ROBOT_TO_CAMERA_TRANSFORMS[i].getRotation().getMeasureX().in(Degrees),
            ROBOT_TO_CAMERA_TRANSFORMS[i].getRotation().getMeasureY().in(Degrees),
            ROBOT_TO_CAMERA_TRANSFORMS[i].getRotation().getMeasureZ().in(Degrees));
    }
  }

  /**
   * Resets the robot's pose to the given new pose.
   * 
   * @param newPose the new pose to reset to (this is absolute and will not be flipped based on the alliance)
   */
  public void resetPose(Pose2d newPose) {
    poseResetConsumer.accept(newPose);
  }

  /**
   * Resets the robot's pose to the given new pose.
   * 
   * @param newPose the new pose to reset to (this is absolute and will not be flipped based on the alliance)
   */
  public void resetPose(Pose3d newPose) {
    poseResetConsumer.accept(newPose.toPose2d());
  }

  /**
   * Periodically updates the localization system with vision and pose data.
   * <p>
   * This method is called automatically by the scheduler. It retrieves and validates vision-based pose estimates from
   * Limelight and QuestNav, and provides vision measurements to the consumer.
   */
  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      periodicDisabled();
    }
    periodicLimelight();
  }

  /**
   * Resets the robot's pose using the best available AprilTag pose estimate from the Limelight cameras
   * <p>
   * This isn't intended for regular use, it's for resetting for practice or if something goes very wrong
   */
  public void resetPoseFromAprilTags() {
    PoseEstimate bestAprilTagPose = null;
    double bestDeviation = Double.MAX_VALUE;

    for (String cameraName : APRILTAG_CAMERA_NAMES) {
      PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
      if (isValidPoseEstimate(poseEstimate)) {
        double adjustedXYDeviation = APRILTAG_TRANSLATION_STD_DEV + (0.01 * Math.pow(poseEstimate.avgTagDist, 2));
        if (bestDeviation > adjustedXYDeviation) {
          bestAprilTagPose = poseEstimate;
          bestDeviation = adjustedXYDeviation;
        }
      }
    }

    if (bestAprilTagPose != null) {
      resetPose(bestAprilTagPose.pose.toPose2d());
    }
  }

  /**
   * Validates a pose estimate based on common criteria.
   * <p>
   * Checks if the pose has valid tag count, is within field boundaries, and is within the tag distance threshold.
   *
   * @param poseEstimate the pose estimate to validate
   * @return true if the pose estimate meets basic validation criteria
   */
  private boolean isValidPoseEstimate(PoseEstimate poseEstimate) {
    return poseEstimate != null && poseEstimate.tagCount > 0
        && isValidFieldTranslation(poseEstimate.pose.getTranslation()) && poseEstimate.tagCount > 1
        && poseEstimate.avgTagDist < TAG_DISTANCE_THRESHOLD.in(Meters);
  }

  /**
   * Handles periodic updates when the robot is disabled.
   */
  private void periodicDisabled() {
    boolean isBlueAlliance = DriverStation.getAlliance().orElse(Blue) == Blue;
    for (String cameraName : APRILTAG_CAMERA_NAMES) {
      // Set the pipeline based on alliance color
      if (isBlueAlliance) {
        LimelightHelpers.setPipelineIndex(cameraName, LIMELIGHT_BLUE_PIPELINE);
      } else {
        LimelightHelpers.setPipelineIndex(cameraName, LIMELIGHT_RED_PIPELINE);
      }
    }
  }

  /**
   * Handles periodic updates when the robot is enabled.
   * <p>
   * Processes all AprilTag cameras, validates poses, adds vision measurements (when QuestNav has failed),
   * and returns the best pose estimate for comparison with QuestNav.
   *
   * @return the best validated pose estimate from all cameras, or null if no valid estimates
   */
  private void periodicLimelight() {

    for (String cameraName : APRILTAG_CAMERA_NAMES) {
      PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

      if (isValidPoseEstimate(poseEstimate)
          && robotAngularVelocitySupplier.get().gt(ANGULAR_VELOCITY_THRESHOLD.unaryMinus())
          && robotAngularVelocitySupplier.get().lt(ANGULAR_VELOCITY_THRESHOLD)) {

        double adjustedXYDeviation = APRILTAG_TRANSLATION_STD_DEV + (0.01 * Math.pow(poseEstimate.avgTagDist, 2));
        // QuestNav is considered unhealthy, fall back to LimeLight measurements
        Matrix<N3, N1> adjustedDeviations = VecBuilder.fill(adjustedXYDeviation, adjustedXYDeviation, Double.MAX_VALUE);
        visionMeasurementConsumer
            .addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds, adjustedDeviations);
      }
    }
  }

}
