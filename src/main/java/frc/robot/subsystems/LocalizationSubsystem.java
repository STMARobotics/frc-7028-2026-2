package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static frc.robot.Constants.FieldConstants.isValidFieldTranslation;
import static frc.robot.Constants.QuestNavConstants.QUESTNAV_FAILURE_THRESHOLD;
import static frc.robot.Constants.QuestNavConstants.QUESTNAV_STD_DEVS;
import static frc.robot.Constants.QuestNavConstants.ROBOT_TO_QUEST;
import static frc.robot.Constants.VisionConstants.ANGULAR_VELOCITY_THRESHOLD;
import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_NAMES;
import static frc.robot.Constants.VisionConstants.APRILTAG_STD_DEVS;
import static frc.robot.Constants.VisionConstants.LIMELIGHT_BLUE_PIPELINE;
import static frc.robot.Constants.VisionConstants.LIMELIGHT_RED_PIPELINE;
import static frc.robot.Constants.VisionConstants.QUESTNAV_APRILTAG_ERROR_THRESHOLD;
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
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
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

  private final QuestNav questNav = new QuestNav();
  private final VisionMeasurementConsumer visionMeasurementConsumer;
  private final Consumer<Pose2d> poseResetConsumer;

  private final Supplier<AngularVelocity> robotAngularVelocitySupplier;
  @Logged
  private double questNavFaultCounter = 0.0;
  // Last quest robot pose. Only an instance variable so it will be logged by Epilogue
  @Logged
  private Pose3d questRobotPose;
  // Last best limelight pose. Only an instance variable so it will be logged by Epilogue
  @Logged
  private Pose3d bestLimelightPose;

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
    setQuestNavPose(newPose);
    poseResetConsumer.accept(newPose);
  }

  /**
   * Resets the robot's pose to the given new pose.
   * 
   * @param newPose the new pose to reset to (this is absolute and will not be flipped based on the alliance)
   */
  public void resetPose(Pose3d newPose) {
    setQuestNavPose(newPose);
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
    PoseEstimate bestEstimate = periodicLimelight();
    periodicQuestNav(bestEstimate);
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
        double adjustedXYDeviation = APRILTAG_STD_DEVS + (0.01 * Math.pow(poseEstimate.avgTagDist, 2));
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
  private PoseEstimate periodicLimelight() {
    PoseEstimate bestEstimate = null;
    double bestDeviation = Double.MAX_VALUE;

    for (String cameraName : APRILTAG_CAMERA_NAMES) {
      PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

      if (isValidPoseEstimate(poseEstimate)
          && robotAngularVelocitySupplier.get().gt(ANGULAR_VELOCITY_THRESHOLD.unaryMinus())
          && robotAngularVelocitySupplier.get().lt(ANGULAR_VELOCITY_THRESHOLD)) {

        double adjustedXYDeviation = APRILTAG_STD_DEVS + (0.01 * Math.pow(poseEstimate.avgTagDist, 2));
        // QuestNav is considered unhealthy, fall back to LimeLight measurements
        Matrix<N3, N1> adjustedDeviations = VecBuilder.fill(adjustedXYDeviation, adjustedXYDeviation, Double.MAX_VALUE);
        visionMeasurementConsumer
            .addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds, adjustedDeviations);

        // Track the best estimate for QuestNav comparison
        if (bestDeviation > adjustedXYDeviation) {
          bestEstimate = poseEstimate;
          bestDeviation = adjustedXYDeviation;
        }
      }
    }
    bestLimelightPose = bestEstimate == null ? null : bestEstimate.pose;

    return bestEstimate;
  }

  /**
   * Handles periodic updates for QuestNav pose estimation.
   * <p>
   * Processes unread QuestNav pose frames, validates poses against AprilTag estimates,
   * manages fault detection, and adds valid vision measurements.
   *
   * @param bestVisionEstimate the best AprilTag pose estimate for comparison, or null if none available
   */
  private void periodicQuestNav(PoseEstimate bestVisionEstimate) {
    questNav.commandPeriodic();
    boolean isQuestWorking = questNav.isTracking() && questNav.isConnected();
    if (!isQuestWorking && questNavFaultCounter < QUESTNAV_FAILURE_THRESHOLD) {
      questNavFaultCounter++;
    }
    PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
    // Iterate backwards through frames to find the most recent valid frame
    for (int i = frames.length - 1; i >= 0; i--) {
      PoseFrame frame = frames[i];
      if (frame.isTracking()) {
        questRobotPose = frame.questPose3d().transformBy(ROBOT_TO_QUEST.inverse());

        double bestVisionEstimateDistance = (bestVisionEstimate != null)
            ? questRobotPose.toPose2d()
                .getTranslation()
                .getDistance(bestVisionEstimate.pose.toPose2d().getTranslation())
            : 0;

        if (bestVisionEstimateDistance > QUESTNAV_APRILTAG_ERROR_THRESHOLD.in(Meters)) {
          // QuestNav disagrees with AprilTag vision - increment fault counter
          if (questNavFaultCounter < QUESTNAV_FAILURE_THRESHOLD) {
            questNavFaultCounter += Math.pow(bestVisionEstimateDistance, 2);
          }
        } else if (bestVisionEstimate != null) {
          // QuestNav agrees with AprilTag vision - decrement fault counter to allow recovery
          questNavFaultCounter = Math.max(questNavFaultCounter - 1.0, 0.0);
        }

        // Only use QuestNav measurements when fault counter is below threshold and pose is valid
        if (questNavFaultCounter < QUESTNAV_FAILURE_THRESHOLD
            && isValidFieldTranslation(questRobotPose.getTranslation())) {
          visionMeasurementConsumer
              .addVisionMeasurement(questRobotPose.toPose2d(), frame.dataTimestamp(), QUESTNAV_STD_DEVS);
        }
        break; // Found the most recent tracking frame, exit loop
      }
    }
  }

  /**
   * Sets the QuestNav pose from the given robot pose.
   * 
   * @param robotPose robot pose
   */
  private void setQuestNavPose(Pose3d robotPose) {
    Pose3d questPose = robotPose.transformBy(ROBOT_TO_QUEST);
    questNav.setPose(questPose);
  }

  /**
   * Sets the QuestNav pose from the given 2D robot pose.
   * <p>
   * This sets the 3D pose with a Z of 0 (on the floor) and no pitch or roll.
   *
   * @param pose2d the robot's 2D pose
   */
  private void setQuestNavPose(Pose2d pose2d) {
    setQuestNavPose(new Pose3d(pose2d));
  }

  @Logged
  public boolean isQuestConnectedAndTracking() {
    return questNav.isConnected() && questNav.isTracking();
  }

  @Logged
  public boolean isQuestBelowErrorThreshold() {
    return questNavFaultCounter < QUESTNAV_FAILURE_THRESHOLD;
  }

}
