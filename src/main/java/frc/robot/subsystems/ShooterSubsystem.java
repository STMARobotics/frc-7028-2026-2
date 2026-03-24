package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static com.ctre.phoenix6.signals.NeutralModeValue.Coast;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_FOLLOWER_MOTOR_ID;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_LEADER_MOTOR_ID;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_PEAK_TORQUE_CURRENT_FORWARD;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_PEAK_TORQUE_CURRENT_REVERSE;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_SLOT_CONFIGS;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_VELOCITY_TOLERANCE;
import static frc.robot.Constants.ShooterConstants.FUEL_EXIT_ANGLE_OFFSET;
import static frc.robot.Constants.ShooterConstants.PITCH_ENCODER_ID;
import static frc.robot.Constants.ShooterConstants.PITCH_HOME_ANGLE;
import static frc.robot.Constants.ShooterConstants.PITCH_LIMIT_FORWARD;
import static frc.robot.Constants.ShooterConstants.PITCH_LIMIT_REVERSE;
import static frc.robot.Constants.ShooterConstants.PITCH_MAGNETIC_OFFSET;
import static frc.robot.Constants.ShooterConstants.PITCH_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.ShooterConstants.PITCH_MOTOR_ID;
import static frc.robot.Constants.ShooterConstants.PITCH_POSITION_TOLERANCE;
import static frc.robot.Constants.ShooterConstants.PITCH_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.ShooterConstants.PITCH_SENSOR_TO_MECHANISM_RATIO;
import static frc.robot.Constants.ShooterConstants.PITCH_SLOT_CONFIGS;
import static frc.robot.Constants.ShooterConstants.PITCH_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.PITCH_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.ROBOT_TO_SHOOTER;
import static frc.robot.Constants.ShooterConstants.YAW_ENCODER_DISCONTINUITY_POINT;
import static frc.robot.Constants.ShooterConstants.YAW_ENCODER_ID;
import static frc.robot.Constants.ShooterConstants.YAW_HOME_ANGLE;
import static frc.robot.Constants.ShooterConstants.YAW_LIMIT_FORWARD;
import static frc.robot.Constants.ShooterConstants.YAW_LIMIT_REVERSE;
import static frc.robot.Constants.ShooterConstants.YAW_MAGNETIC_OFFSET;
import static frc.robot.Constants.ShooterConstants.YAW_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.ShooterConstants.YAW_MOTOR_ID;
import static frc.robot.Constants.ShooterConstants.YAW_POSITION_TOLERANCE;
import static frc.robot.Constants.ShooterConstants.YAW_RANGE_FORWARD;
import static frc.robot.Constants.ShooterConstants.YAW_RANGE_REVERSE;
import static frc.robot.Constants.ShooterConstants.YAW_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.ShooterConstants.YAW_SENSOR_TO_MECHANISM_RATIO;
import static frc.robot.Constants.ShooterConstants.YAW_SLOT_CONFIGS;
import static frc.robot.Constants.ShooterConstants.YAW_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.YAW_SUPPLY_CURRENT_LIMIT;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/** Shooter subsystem: turret yaw + pitch + flywheel. */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX yawMotor = new TalonFX(YAW_MOTOR_ID, CANIVORE_BUS);
  private final CANcoder yawEncoder = new CANcoder(YAW_ENCODER_ID, CANIVORE_BUS);

  private final TalonFX pitchMotor = new TalonFX(PITCH_MOTOR_ID, CANIVORE_BUS);
  private final CANcoder pitchEncoder = new CANcoder(PITCH_ENCODER_ID, CANIVORE_BUS);

  private final TalonFX flywheelLeaderMotor = new TalonFX(FLYWHEEL_LEADER_MOTOR_ID, CANIVORE_BUS);
  private final TalonFX flywheelFollowerMotor = new TalonFX(FLYWHEEL_FOLLOWER_MOTOR_ID, CANIVORE_BUS);

  private final MotionMagicVoltage yawPositionRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final MotionMagicVoltage pitchPositionRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0.0);

  private final VoltageOut sysIdYawVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut sysIdPitchVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC sysIdFlywheelTorqueCurrent = new TorqueCurrentFOC(0.0);

  private final StatusSignal<Angle> yawPosition = yawMotor.getPosition();
  private final StatusSignal<AngularVelocity> yawVelocity = yawMotor.getVelocity();
  private final StatusSignal<Angle> pitchPosition = pitchMotor.getPosition();
  private final StatusSignal<AngularVelocity> pitchVelocity = pitchMotor.getVelocity();
  private final StatusSignal<AngularVelocity> flywheelVelocity = flywheelLeaderMotor.getVelocity();
  private final StatusSignal<AngularAcceleration> flywheelAcceleration = flywheelLeaderMotor.getAcceleration();

  // SysId routines
  private final SysIdRoutine yawSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(0.25).per(Second),
          Volts.of(1),
          Seconds.of(10),
          state -> SignalLogger.writeString("Yaw Motor SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> yawMotor.setControl(sysIdYawVoltage.withOutput(volts.in(Volts))),
          null,
          this));

  private final SysIdRoutine pitchSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(0.1).per(Second),
          Volts.of(0.3),
          Seconds.of(10),
          state -> SignalLogger.writeString("Pitch Motor SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> pitchMotor.setControl(sysIdPitchVoltage.withOutput(volts.in(Volts))),
          null,
          this));

  // NOTE: the output type is amps, NOT volts (even though it says volts)
  // https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/8
  private final SysIdRoutine flywheelSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(5).per(Second),
          Volts.of(10),
          Seconds.of(10),
          state -> SignalLogger.writeString("Flywheel SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          amps -> flywheelLeaderMotor.setControl(sysIdFlywheelTorqueCurrent.withOutput(amps.in(Volts))),
          null,
          this));

  /**
   * Creates a new shooter subsystem
   */
  public ShooterSubsystem() {
    configureYaw();
    configurePitch();
    configureFlywheel();
  }

  private void configureYaw() {
    CANcoderConfiguration yawCanCoderConfig = new CANcoderConfiguration().withMagnetSensor(
        new MagnetSensorConfigs().withMagnetOffset(YAW_MAGNETIC_OFFSET)
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(YAW_ENCODER_DISCONTINUITY_POINT));
    yawEncoder.getConfigurator().apply(yawCanCoderConfig);

    TalonFXConfiguration yawTalonConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(Brake))
        .withCurrentLimits(
            new CurrentLimitsConfigs().withSupplyCurrentLimit(YAW_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(YAW_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true))
        .withFeedback(
            new FeedbackConfigs().withRotorToSensorRatio(YAW_ROTOR_TO_SENSOR_RATIO)
                .withFusedCANcoder(yawEncoder)
                .withSensorToMechanismRatio(YAW_SENSOR_TO_MECHANISM_RATIO))
        .withSlot0(Slot0Configs.from(YAW_SLOT_CONFIGS))
        .withMotionMagic(YAW_MOTION_MAGIC_CONFIGS)
        .withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(YAW_LIMIT_FORWARD)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(YAW_LIMIT_REVERSE));

    yawMotor.getConfigurator().apply(yawTalonConfig);
  }

  private void configurePitch() {
    CANcoderConfiguration pitchCanCoderConfig = new CANcoderConfiguration().withMagnetSensor(
        new MagnetSensorConfigs().withMagnetOffset(PITCH_MAGNETIC_OFFSET)
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive));
    pitchEncoder.getConfigurator().apply(pitchCanCoderConfig);

    TalonFXConfiguration pitchTalonConfig = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(Brake))
        .withCurrentLimits(
            new CurrentLimitsConfigs().withStatorCurrentLimit(PITCH_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(PITCH_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true))
        .withFeedback(
            new FeedbackConfigs().withRotorToSensorRatio(PITCH_ROTOR_TO_SENSOR_RATIO)
                .withSensorToMechanismRatio(PITCH_SENSOR_TO_MECHANISM_RATIO)
                .withFusedCANcoder(pitchEncoder))
        .withSlot0(Slot0Configs.from(PITCH_SLOT_CONFIGS))
        .withMotionMagic(PITCH_MOTION_MAGIC_CONFIGS)
        .withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(PITCH_LIMIT_FORWARD)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(PITCH_LIMIT_REVERSE));

    pitchMotor.getConfigurator().apply(pitchTalonConfig);
  }

  private void configureFlywheel() {
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs().withNeutralMode(Coast).withInverted(InvertedValue.CounterClockwise_Positive))

        .withTorqueCurrent(
            new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(FLYWHEEL_PEAK_TORQUE_CURRENT_FORWARD)
                .withPeakReverseTorqueCurrent(FLYWHEEL_PEAK_TORQUE_CURRENT_REVERSE))
        .withCurrentLimits(
            new CurrentLimitsConfigs().withStatorCurrentLimit(FLYWHEEL_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(FLYWHEEL_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true))
        .withSlot0(Slot0Configs.from(FLYWHEEL_SLOT_CONFIGS));

    flywheelLeaderMotor.getConfigurator().apply(flywheelConfig);
    flywheelFollowerMotor.getConfigurator().apply(flywheelConfig);
    // Max the leader update frequency so follower can respond quickly
    flywheelLeaderMotor.getTorqueCurrent().setUpdateFrequency(1000);

    flywheelFollowerMotor.setControl(new Follower(flywheelLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  /**
   * Builds a command that runs yaw SysId in quasistatic mode.
   *
   * @param direction direction for the SysId sweep
   * @return command that runs yaw quasistatic SysId and stops yaw on exit
   */
  public Command sysIdYawQuasistaticCommand(Direction direction) {
    return yawSysIdRoutine.quasistatic(direction).withName("SysId yaw quasi " + direction).finallyDo(this::stopYaw);
  }

  /**
   * Builds a command that runs yaw SysId in dynamic mode.
   *
   * @param direction direction for the SysId sweep
   * @return command that runs yaw dynamic SysId and stops yaw on exit
   */
  public Command sysIdYawDynamicCommand(Direction direction) {
    return yawSysIdRoutine.dynamic(direction).withName("SysId yaw dynamic " + direction).finallyDo(this::stopYaw);
  }

  /**
   * Builds a command that runs pitch SysId in quasistatic mode.
   *
   * @param direction direction for the SysId sweep
   * @return command that runs pitch quasistatic SysId and stops pitch on exit
   */
  public Command sysIdPitchQuasistaticCommand(Direction direction) {
    return pitchSysIdRoutine.quasistatic(direction)
        .withName("SysId pitch quasi " + direction)
        .finallyDo(this::stopPitch);
  }

  /**
   * Builds a command that runs pitch SysId in dynamic mode.
   *
   * @param direction direction for the SysId sweep
   * @return command that runs pitch dynamic SysId and stops pitch on exit
   */
  public Command sysIdPitchDynamicCommand(Direction direction) {
    return pitchSysIdRoutine.dynamic(direction).withName("SysId pitch dynamic " + direction).finallyDo(this::stopPitch);
  }

  /**
   * Builds a command that runs flywheel SysId in quasistatic mode.
   *
   * @param direction direction for the SysId sweep
   * @return command that runs flywheel quasistatic SysId and stops flywheel on exit
   */
  public Command sysIdFlywheelQuasistaticCommand(Direction direction) {
    return flywheelSysIdRoutine.quasistatic(direction)
        .withName("SysId flywheel quasi " + direction)
        .finallyDo(this::stopFlywheel);
  }

  /**
   * Builds a command that runs flywheel SysId in dynamic mode.
   *
   * @param direction direction for the SysId sweep
   * @return command that runs flywheel dynamic SysId and stops flywheel on exit
   */
  public Command sysIdFlywheelDynamicCommand(Direction direction) {
    return flywheelSysIdRoutine.dynamic(direction)
        .withName("SysId flywheel dynamic " + direction)
        .finallyDo(this::stopFlywheel);
  }

  /**
   * Commands yaw to an angle
   *
   * <p>
   * Input yaw may be wrapped or continuous. Since the turret turns more than one rotation, this will choose the
   * nearest equivalent yaw that it can reach.
   *
   * @param targetYaw requested yaw
   */
  public void setYawAngle(Angle targetYaw) {
    // Wrap the input to match the range of the turret. The input is probably in the range of (-0.5, 0.5], but the
    // turret range is more like [-0.75, 0.25].
    double targetRotations = MathUtil
        .inputModulus(targetYaw.in(Rotations), YAW_RANGE_REVERSE.in(Rotations), YAW_RANGE_FORWARD.in(Rotations));
    Angle currentYaw = getYaw();
    double ffVolts = 0.0;
    if (currentYaw.gt(Rotations.of(-0.195))) {
      ffVolts = 0.5;
    } else if (currentYaw.lt(Rotations.of(-0.468))) {
      ffVolts = -1.0375;
    }
    yawMotor.setControl(yawPositionRequest.withPosition(targetRotations).withFeedForward(Volts.of(ffVolts)));
  }

  /**
   * Commands the pitch to an angle
   *
   * @param targetPitch requested pitch
   */
  public void setPitchAngle(Angle targetPitch) {
    pitchMotor.setControl(pitchPositionRequest.withPosition(targetPitch));
  }

  /**
   * Commands flywheel to a velocity
   *
   * @param targetSpeed desired flywheel velocity
   */
  public void setFlywheelSpeed(AngularVelocity flywheelVelocity) {
    flywheelLeaderMotor.setControl(flywheelVelocityRequest.withVelocity(flywheelVelocity));
  }

  /**
   * Applies pitch and flywheel setpoints
   *
   * @param setpoints shooter setpoints
   */
  public void applySetpoints(ShooterSetpoints setpoints) {
    setPitchAngle(setpoints.targetPitch());
    setFlywheelSpeed(setpoints.targetFlywheelSpeed());
  }

  /** Moves yaw/pitch to home angles and stops the flywheel */
  public void stow() {
    // setYawAngle(YAW_HOME_ANGLE);
    setPitchAngle(PITCH_HOME_ANGLE);
    flywheelLeaderMotor.stopMotor();
  }

  /**
   * Moves the pitch to the stow position so it fits under the trench
   */
  public void stowPitch() {
    setPitchAngle(PITCH_HOME_ANGLE);
  }

  /** Stops the yaw */
  public void stopYaw() {
    yawMotor.stopMotor();
  }

  /** Stops the pitch */
  public void stopPitch() {
    pitchMotor.stopMotor();
  }

  /** Stops the flywheel */
  public void stopFlywheel() {
    flywheelLeaderMotor.stopMotor();
  }

  /** Stops yaw, pitch, and flywheel */
  public void stopAll() {
    stopYaw();
    stopPitch();
    stopFlywheel();
  }

  /**
   * Returns unwrapped turret yaw
   *
   * @return current turret yaw
   */
  @Logged(name = "Yaw")
  public Angle getYaw() {
    BaseStatusSignal.refreshAll(yawPosition, yawVelocity);
    return BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity);
  }

  /**
   * Gets the shooter's yaw angular velocity.
   * 
   * @return yaw angular velocity
   */
  public AngularVelocity getYawVelocity() {
    return yawVelocity.refresh().getValue();
  }

  /**
   * Returns current turret pitch
   *
   * @return pitch
   */
  @Logged(name = "Pitch")
  public Angle getPitch() {
    BaseStatusSignal.refreshAll(pitchPosition, pitchVelocity);
    return BaseStatusSignal.getLatencyCompensatedValue(pitchPosition, pitchVelocity);
  }

  /**
   * Returns current flywheel velocity
   *
   * @return flywheel angular velocity
   */
  @Logged(name = "Flywheel Velocity")
  public AngularVelocity getFlywheelVelocity() {
    BaseStatusSignal.refreshAll(flywheelVelocity, flywheelAcceleration);
    return BaseStatusSignal.getLatencyCompensatedValue(flywheelVelocity, flywheelAcceleration);
  }

  /** Returns whether current yaw is within tolerance of the active yaw request. */
  @Logged
  public boolean isYawAtSetpoint() {
    BaseStatusSignal.refreshAll(yawPosition, yawVelocity);
    Angle currentYaw = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity);
    return MathUtil.isNear(yawPositionRequest.Position, currentYaw.in(Rotations), YAW_POSITION_TOLERANCE.in(Rotations));
  }

  /** Returns whether pitch is within configured tolerance of the active request. */
  @Logged
  public boolean isPitchAtSetpoint() {
    BaseStatusSignal.refreshAll(pitchPosition, pitchVelocity);
    Angle currentPitch = BaseStatusSignal.getLatencyCompensatedValue(pitchPosition, pitchVelocity);
    return MathUtil
        .isNear(pitchPositionRequest.Position, currentPitch.in(Rotations), PITCH_POSITION_TOLERANCE.in(Rotations));
  }

  /** Returns whether flywheel speed is within configured tolerance of the active request. */
  @Logged
  public boolean isFlywheelAtSpeed() {
    BaseStatusSignal.refreshAll(flywheelVelocity, flywheelAcceleration);
    AngularVelocity currentSpeed = BaseStatusSignal.getLatencyCompensatedValue(flywheelVelocity, flywheelAcceleration);
    return MathUtil.isNear(
        flywheelVelocityRequest.Velocity,
          currentSpeed.in(RotationsPerSecond),
          FLYWHEEL_VELOCITY_TOLERANCE.in(RotationsPerSecond));
  }

  @Logged
  public boolean isPitchStowed() {
    BaseStatusSignal.refreshAll(pitchPosition, pitchVelocity);
    Angle currentPitch = BaseStatusSignal.getLatencyCompensatedValue(pitchPosition, pitchVelocity);
    return MathUtil
        .isNear(PITCH_HOME_ANGLE.in(Rotations), currentPitch.in(Rotations), PITCH_POSITION_TOLERANCE.in(Rotations));
  }

  @Logged
  public boolean isYawStowed() {
    BaseStatusSignal.refreshAll(yawPosition, yawVelocity);
    Angle currentYaw = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity);
    return MathUtil
        .isNear(YAW_HOME_ANGLE.in(Rotations), currentYaw.in(Rotations), YAW_POSITION_TOLERANCE.in(Rotations));
  }

  @Logged
  public boolean isStowed() {
    return isPitchStowed() && isYawStowed();
  }

  /**
   * Checks if the shooter is ready to shoot by verifying that yaw and pitch are at their setpoints and the flywheel is
   * at velocity.
   * 
   * @return true if shooter is ready to shoot, false otherwise
   */
  @Logged
  public boolean isReadyToShoot() {
    return isYawAtSetpoint() && isPitchAtSetpoint() && isFlywheelAtSpeed();
  }

  /**
   * Gets the translation of the turret center in field coordinates.
   * 
   * @param robotPose the current pose of the robot
   * @return the translation of the turret center in field coordinates
   */
  public static Translation2d getShooterTranslation(Pose2d robotPose) {
    return robotPose.getTranslation().plus(ROBOT_TO_SHOOTER.rotateBy(robotPose.getRotation()));
  }

  /**
   * Gets an estimated pitch of fuel exiting the shooter
   * 
   * @return fuel pitch angle
   */
  public static Angle getFuelPitch(Angle shooterPitch) {
    // The hood moving up (positive) lowers the exit angle, so the angle has to be subtracted.
    // Turret 0 shoots fuel at FUEL_EXIT_ANGLE_OFFSET
    return FUEL_EXIT_ANGLE_OFFSET.minus(shooterPitch);
  }

  /**
   * Setpoints for the shooter subsystem.
   * 
   * @param targetPitch pitch angle
   * @param targetFlywheelSpeed flywheel velocity
   * @param spindexerVelocity spindexer velocity
   * @param feederVelocity feeder velocity
   */
  public static record ShooterSetpoints(
      Angle targetPitch,
      AngularVelocity targetFlywheelSpeed,
      AngularVelocity spindexerVelocity,
      AngularVelocity feederVelocity) {
    /**
     * Interpolates between this and another ShootingSettings.
     *
     * @param endValue the end value
     * @param t the interpolation parameter [0, 1]
     * @return interpolated ShootingSettings
     */
    public ShooterSetpoints interpolate(ShooterSetpoints endValue, double t) {
      ShooterSetpoints result = new ShooterSetpoints(
          Rotations.of(MathUtil.interpolate(targetPitch.in(Rotations), endValue.targetPitch.in(Rotations), t)),
          RotationsPerSecond.of(
              MathUtil.interpolate(
                  targetFlywheelSpeed.in(RotationsPerSecond),
                    endValue.targetFlywheelSpeed.in(RotationsPerSecond),
                    t)),
          RotationsPerSecond.of(
              MathUtil.interpolate(
                  spindexerVelocity.in(RotationsPerSecond),
                    endValue.spindexerVelocity.in(RotationsPerSecond),
                    t)),
          RotationsPerSecond.of(
              MathUtil.interpolate(
                  feederVelocity.in(RotationsPerSecond),
                    endValue.feederVelocity.in(RotationsPerSecond),
                    t)));
      return result;
    }
  }
}
