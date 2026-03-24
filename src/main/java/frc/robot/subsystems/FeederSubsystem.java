package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS;
import static frc.robot.Constants.FeederConstants.DEVICE_ID_FEEDER_CANRANGE;
import static frc.robot.Constants.FeederConstants.DEVICE_ID_FEEDER_MOTOR;
import static frc.robot.Constants.FeederConstants.FEEDER_FEED_VELOCITY;
import static frc.robot.Constants.FeederConstants.FEEDER_PEAK_TORQUE_CURRENT_FORWARD;
import static frc.robot.Constants.FeederConstants.FEEDER_PEAK_TORQUE_CURRENT_REVERSE;
import static frc.robot.Constants.FeederConstants.FEEDER_SLOT_CONFIGS;
import static frc.robot.Constants.FeederConstants.FEEDER_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.FeederConstants.FEEDER_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.FeederConstants.FEEDER_UNJAM_VELOCITY;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * Subsystem for the Feeder.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class FeederSubsystem extends SubsystemBase {
  private final TalonFX feederMotor = new TalonFX(DEVICE_ID_FEEDER_MOTOR, CANIVORE_BUS);
  private final CANrange feederCanRange = new CANrange(DEVICE_ID_FEEDER_CANRANGE, CANIVORE_BUS);

  private final VelocityTorqueCurrentFOC feederVelocityTorque = new VelocityTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC feederTorqueControl = new TorqueCurrentFOC(0.0);

  private final StatusSignal<Boolean> feederBallSignal = feederCanRange.getIsDetected();

  // NOTE: the output type is amps, NOT volts (even though it says volts)
  // https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/8
  private final SysIdRoutine feederSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(3.0).per(Second),
          Volts.of(25),
          null,
          state -> SignalLogger.writeString("Feeder SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          amps -> feederMotor.setControl(feederTorqueControl.withOutput(amps.in(Volts))),
          null,
          this));

  public FeederSubsystem() {
    CANrangeConfiguration feederCanRangeConfig = new CANrangeConfiguration()
        .withProximityParams(new ProximityParamsConfigs().withProximityThreshold(0.1));
    feederCanRange.getConfigurator().apply(feederCanRangeConfig);

    var feederTalonconfig = new TalonFXConfiguration().withSlot0(Slot0Configs.from(FEEDER_SLOT_CONFIGS))
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
        .withTorqueCurrent(
            new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(FEEDER_PEAK_TORQUE_CURRENT_FORWARD)
                .withPeakReverseTorqueCurrent(FEEDER_PEAK_TORQUE_CURRENT_REVERSE))
        .withCurrentLimits(
            new CurrentLimitsConfigs().withStatorCurrentLimit(FEEDER_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(FEEDER_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true));
    feederTalonconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    feederMotor.getConfigurator().apply(feederTalonconfig);
  }

  public Command sysIdFeederDynamicCommand(Direction direction) {
    return feederSysIdRoutine.dynamic(direction).withName("SysId feeder dynamic " + direction).finallyDo(this::stop);
  }

  public Command sysIdFeederQuasistaticCommand(Direction direction) {
    return feederSysIdRoutine.quasistatic(direction).withName("SysId feeder quasi " + direction).finallyDo(this::stop);
  }

  /**
   * Spins the feeder to feed the shooter
   */
  public void feedShooter() {
    feederMotor.setControl(feederVelocityTorque.withVelocity(FEEDER_FEED_VELOCITY));
  }

  /**
   * Run the feeder at the set velocity. Used for tuning, should not be used for normal operation.
   * 
   * @param velocity the velocity to run the feeder
   */
  public void runFeeder(AngularVelocity velocity) {
    feederMotor.setControl(feederVelocityTorque.withVelocity(velocity));
  }

  /**
   * Spins the feeder backward to unjam the feeder
   */
  public void unjam() {
    feederMotor.setControl(feederVelocityTorque.withVelocity(FEEDER_UNJAM_VELOCITY));
  }

  /**
   * Stops the feeder motor
   */
  public void stop() {
    feederMotor.stopMotor();
  }

  /**
   * Returns true if the feeder has a ball in it
   */
  @Logged
  public boolean isFull() {
    return feederBallSignal.refresh().getValue();
  }

  /**
   * Returns true if the feeder is empty
   */
  @Logged
  public boolean isEmpty() {
    return !isFull();
  }
}
