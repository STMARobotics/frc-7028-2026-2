package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS;
import static frc.robot.Constants.IndexerConstants.DEVICE_ID_INDEXER_MOTOR;
import static frc.robot.Constants.IndexerConstants.INDEXER_FEED_VELOCITY;
import static frc.robot.Constants.IndexerConstants.INDEXER_PEAK_TORQUE_CURRENT_FORWARD;
import static frc.robot.Constants.IndexerConstants.INDEXER_PEAK_TORQUE_CURRENT_REVERSE;
import static frc.robot.Constants.IndexerConstants.INDEXER_SLOT_CONFIGS;
import static frc.robot.Constants.IndexerConstants.INDEXER_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.IndexerConstants.INDEXER_SUPPLY_CURRENT_LIMIT;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
 * Subsystem for the Indexer.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class IndexerSubsystem extends SubsystemBase {

  private final TalonFX indexerMotor = new TalonFX(DEVICE_ID_INDEXER_MOTOR, CANIVORE_BUS);

  private final VelocityTorqueCurrentFOC indexerVelocityTorque = new VelocityTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC indexerTorqueControl = new TorqueCurrentFOC(0.0);

  // NOTE: the output type is amps, NOT volts (even though it says volts)
  // https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/8
  private final SysIdRoutine indexerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(3.0).per(Second),
          Volts.of(25),
          null,
          state -> SignalLogger.writeString("Indexer SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          amps -> indexerMotor.setControl(indexerTorqueControl.withOutput(amps.in(Volts))),
          null,
          this));

  /**
   * Creates a new Subsystem for the Indexer
   */
  public IndexerSubsystem() {
    var dexTalonconfig = new TalonFXConfiguration().withSlot0(Slot0Configs.from(INDEXER_SLOT_CONFIGS))
        .withMotorOutput(
            new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast))
        .withTorqueCurrent(
            new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(INDEXER_PEAK_TORQUE_CURRENT_FORWARD)
                .withPeakReverseTorqueCurrent(INDEXER_PEAK_TORQUE_CURRENT_REVERSE))
        .withCurrentLimits(
            new CurrentLimitsConfigs().withStatorCurrentLimit(INDEXER_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(INDEXER_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true));

    indexerMotor.getConfigurator().apply(dexTalonconfig);
  }

  public Command sysIdIndexerDynamicCommand(Direction direction) {
    return indexerSysIdRoutine.dynamic(direction).withName("SysId indexer dynamic " + direction).finallyDo(this::stop);
  }

  public Command sysIdIndexerQuasistaticCommand(Direction direction) {
    return indexerSysIdRoutine.quasistatic(direction)
        .withName("SysId indexer quasi " + direction)
        .finallyDo(this::stop);
  }

  /**
   * Spins the indexer forward to feed the shooter
   */
  public void feedShooter() {
    indexerMotor.setControl(indexerVelocityTorque.withVelocity(INDEXER_FEED_VELOCITY));
  }

  /**
   * Run the indexer at the set velocity. Used for tuning, should not be used for normal operation.
   * 
   * @param velocity the velocity to run the indexer
   */
  public void runIndexer(AngularVelocity velocity) {
    indexerMotor.setControl(indexerVelocityTorque.withVelocity(velocity));
  }

  /**
   * Stops the indexer
   */
  public void stop() {
    indexerMotor.stopMotor();
  }
}