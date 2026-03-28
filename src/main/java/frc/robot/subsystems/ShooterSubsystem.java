package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Coast;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_FLYWHEEL_FOLLOWER_0;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_FLYWHEEL_FOLLOWER_1;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_FLYWHEEL_FOLLOWER_2;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_FLYWHEEL_LEADER;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_PEAK_TORQUE_CURRENT_FORWARD;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_PEAK_TORQUE_CURRENT_REVERSE;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_SLOT_CONFIGS;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_VELOCITY_TOLERANCE;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/** Shooter subsystem: turret yaw + pitch + flywheel. */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX flywheelLeaderMotor = new TalonFX(DEVICE_ID_FLYWHEEL_LEADER, CANIVORE_BUS);
  private final TalonFX flywheelFollower0Motor = new TalonFX(DEVICE_ID_FLYWHEEL_FOLLOWER_0, CANIVORE_BUS);
  private final TalonFX flywheelFollower1Motor = new TalonFX(DEVICE_ID_FLYWHEEL_FOLLOWER_1, CANIVORE_BUS);
  private final TalonFX flywheelFollower2Motor = new TalonFX(DEVICE_ID_FLYWHEEL_FOLLOWER_2, CANIVORE_BUS);

  private final VelocityTorqueCurrentFOC flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0.0);

  private final TorqueCurrentFOC sysIdFlywheelTorqueCurrent = new TorqueCurrentFOC(0.0);

  private final StatusSignal<AngularVelocity> flywheelVelocity = flywheelLeaderMotor.getVelocity();
  private final StatusSignal<AngularAcceleration> flywheelAcceleration = flywheelLeaderMotor.getAcceleration();

  // SysId routines
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
    flywheelFollower0Motor.getConfigurator().apply(flywheelConfig);
    flywheelFollower1Motor.getConfigurator().apply(flywheelConfig);
    flywheelFollower2Motor.getConfigurator().apply(flywheelConfig);
    // Max the leader update frequency so followers can respond quickly
    flywheelLeaderMotor.getTorqueCurrent().setUpdateFrequency(1000);

    flywheelFollower0Motor.setControl(new Follower(flywheelLeaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    flywheelFollower1Motor.setControl(new Follower(flywheelLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    flywheelFollower2Motor.setControl(new Follower(flywheelLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
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
        .finallyDo(this::stop);
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
        .finallyDo(this::stop);
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
   * Stops the shooter
   */
  public void stop() {
    flywheelLeaderMotor.stopMotor();
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

  /**
   * Checks if the shooter is ready to shoot by verifying that yaw and pitch are at their setpoints and the flywheel is
   * at velocity.
   * 
   * @return true if shooter is ready to shoot, false otherwise
   */
  @Logged
  public boolean isReadyToShoot() {
    return isFlywheelAtSpeed();
  }

}
