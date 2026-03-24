package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS;
import static frc.robot.Constants.SpindexerConstants.DEVICE_ID_SPINDEXER_MOTOR;
import static frc.robot.Constants.SpindexerConstants.SPINDEXER_AGITATE_BACKWARD_VELOCITY;
import static frc.robot.Constants.SpindexerConstants.SPINDEXER_AGITATE_FORWARD_VELOCITY;
import static frc.robot.Constants.SpindexerConstants.SPINDEXER_FEED_VELOCITY;
import static frc.robot.Constants.SpindexerConstants.SPINDEXER_PEAK_TORQUE_CURRENT_FORWARD;
import static frc.robot.Constants.SpindexerConstants.SPINDEXER_PEAK_TORQUE_CURRENT_REVERSE;
import static frc.robot.Constants.SpindexerConstants.SPINDEXER_SLOT_CONFIGS;
import static frc.robot.Constants.SpindexerConstants.SPINDEXER_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.SpindexerConstants.SPINDEXER_SUPPLY_CURRENT_LIMIT;

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
 * Subsystem for the Spindexer.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class SpindexerSubsystem extends SubsystemBase {

  private final TalonFX spindexerMotor = new TalonFX(DEVICE_ID_SPINDEXER_MOTOR, CANIVORE_BUS);

  private final VelocityTorqueCurrentFOC spindexerVelocityTorque = new VelocityTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC spindexerTorqueControl = new TorqueCurrentFOC(0.0);

  // NOTE: the output type is amps, NOT volts (even though it says volts)
  // https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/8
  private final SysIdRoutine spindexerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(3.0).per(Second),
          Volts.of(25),
          null,
          state -> SignalLogger.writeString("Spindexer SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          amps -> spindexerMotor.setControl(spindexerTorqueControl.withOutput(amps.in(Volts))),
          null,
          this));

  /**
   * Creates a new Subsystem for the Spindexer
   */
  public SpindexerSubsystem() {
    var spinTalonconfig = new TalonFXConfiguration().withSlot0(Slot0Configs.from(SPINDEXER_SLOT_CONFIGS))
        .withMotorOutput(
            new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast))
        .withTorqueCurrent(
            new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(SPINDEXER_PEAK_TORQUE_CURRENT_FORWARD)
                .withPeakReverseTorqueCurrent(SPINDEXER_PEAK_TORQUE_CURRENT_REVERSE))
        .withCurrentLimits(
            new CurrentLimitsConfigs().withStatorCurrentLimit(SPINDEXER_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(SPINDEXER_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true));

    spindexerMotor.getConfigurator().apply(spinTalonconfig);
  }

  public Command sysIdSpindexerDynamicCommand(Direction direction) {
    return spindexerSysIdRoutine.dynamic(direction)
        .withName("SysId spindexer dynamic " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdSpindexerQuasistaticCommand(Direction direction) {
    return spindexerSysIdRoutine.quasistatic(direction)
        .withName("SysId spindexer quasi " + direction)
        .finallyDo(this::stop);
  }

  /**
   * Spins the spindexer forward to feed the shooter
   */
  public void feedShooter() {
    spindexerMotor.setControl(spindexerVelocityTorque.withVelocity(SPINDEXER_FEED_VELOCITY));
  }

  /**
   * Agitates the spindexer back and forth to prevent jams
   */
  public Command agitate() {
    return run(this::spinForward).withTimeout(0.5)
        .andThen(run(this::spinBackward).withTimeout(0.5))
        .repeatedly()
        .finallyDo(this::stop);
  }

  /**
   * Run the spindexer at the set velocity. Used for tuning, should not be used for normal operation.
   * 
   * @param velocity the velocity to run the spindexer
   */
  public void runSpindexer(AngularVelocity velocity) {
    spindexerMotor.setControl(spindexerVelocityTorque.withVelocity(velocity));
  }

  /**
   * Stops the spindexer
   */
  public void stop() {
    spindexerMotor.stopMotor();
  }

  /**
   * Spins the spindexer forward
   */
  private void spinForward() {
    spindexerMotor.setControl(spindexerVelocityTorque.withVelocity(SPINDEXER_AGITATE_FORWARD_VELOCITY));
  }

  /**
   * Spins the spindexer backward
   */
  private void spinBackward() {
    spindexerMotor.setControl(spindexerVelocityTorque.withVelocity(SPINDEXER_AGITATE_BACKWARD_VELOCITY));
  }

}