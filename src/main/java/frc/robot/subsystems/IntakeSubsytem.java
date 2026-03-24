//Copyright (c) FIRST and other WPILib contributors
//Open Source Software; can modify and/or share it under the terms of
//the WPILib BSD licensefile in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS;
import static frc.robot.Constants.IntakeConstants.DEPLOYED_POSITION;
import static frc.robot.Constants.IntakeConstants.DEPLOY_CANCODER_OFFSET;
import static frc.robot.Constants.IntakeConstants.DEPLOY_DISCONTINUITY_POINT;
import static frc.robot.Constants.IntakeConstants.DEPLOY_FORWARD_LIMIT;
import static frc.robot.Constants.IntakeConstants.DEPLOY_HOLD_CURRENT;
import static frc.robot.Constants.IntakeConstants.DEPLOY_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.IntakeConstants.DEPLOY_REVERSE_LIMIT;
import static frc.robot.Constants.IntakeConstants.DEPLOY_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.IntakeConstants.DEPLOY_SENSOR_TO_MECHANISM_RATIO;
import static frc.robot.Constants.IntakeConstants.DEPLOY_SLOT_CONFIGS;
import static frc.robot.Constants.IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.IntakeConstants.DEPLOY_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.IntakeConstants.DEPLOY_TOLERANCE;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_DEPLOY_CANCODER;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_DEPLOY_MOTOR;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_ROLLER_MOTOR;
import static frc.robot.Constants.IntakeConstants.RETRACTED_POSITION;
import static frc.robot.Constants.IntakeConstants.ROLLER_EJECT_VELOCITY;
import static frc.robot.Constants.IntakeConstants.ROLLER_INTAKE_VELOCITY;
import static frc.robot.Constants.IntakeConstants.ROLLER_PEAK_TORQUE_CURRENT_FORWARD;
import static frc.robot.Constants.IntakeConstants.ROLLER_PEAK_TORQUE_CURRENT_REVERSE;
import static frc.robot.Constants.IntakeConstants.ROLLER_SLOT_CONFIGS;
import static frc.robot.Constants.IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.IntakeConstants.ROLLER_SUPPLY_CURRENT_LIMIT;

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
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * Subsytem for the intake.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class IntakeSubsytem extends SubsystemBase {

  private final TalonFX rollerMotor = new TalonFX(DEVICE_ID_ROLLER_MOTOR, CANIVORE_BUS);
  private final TalonFX deployMotor = new TalonFX(DEVICE_ID_DEPLOY_MOTOR, CANIVORE_BUS);
  private final CANcoder deployCANcoder = new CANcoder(DEVICE_ID_DEPLOY_CANCODER, CANIVORE_BUS);

  // Motor request objects
  private final MotionMagicVoltage deployControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC rollerControl = new VelocityTorqueCurrentFOC(0.0);

  private final TorqueCurrentFOC rollerSysIdControl = new TorqueCurrentFOC(0.0);
  private final VoltageOut deploySysIdControl = new VoltageOut(0.0).withEnableFOC(true);
  // Torque control to hold the intake, ignoring the soft limit
  private final TorqueCurrentFOC deployHoldControl = new TorqueCurrentFOC(DEPLOY_HOLD_CURRENT)
      .withIgnoreSoftwareLimits(true);

  private final StatusSignal<Angle> deployPositionSignal = deployMotor.getPosition(false);
  private final StatusSignal<AngularVelocity> deployVelocitySignal = deployMotor.getVelocity(false);
  private final StatusSignal<Temperature> deployTempSignal = deployMotor.getDeviceTemp(false);
  private final StatusSignal<Boolean> deployTempFaultSignal = deployMotor.getFault_DeviceTemp(false);

  // SysId routines
  // NOTE: the output type is amps, NOT volts (even though it says volts)
  // https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/8
  private final SysIdRoutine rollerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(1).per(Second),
          Volts.of(30),
          null,
          state -> SignalLogger.writeString("Intake Roller SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          (amps) -> rollerMotor.setControl(rollerSysIdControl.withOutput(amps.in(Volts))),
          null,
          this));

  private final SysIdRoutine deploySysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(0.1).per(Second),
          Volts.of(1),
          null,
          state -> SignalLogger.writeString("Intake Deploy SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          (volts) -> deployMotor.setControl(deploySysIdControl.withOutput(volts.in(Volts))),
          null,
          this));

  /**
   * Creates a new substyem for the intake
   */
  public IntakeSubsytem() {
    // Configure the roller motor
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.withMotorOutput(
        new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.Clockwise_Positive))
        .withSlot0(Slot0Configs.from(ROLLER_SLOT_CONFIGS))
        .withTorqueCurrent(
            new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(ROLLER_PEAK_TORQUE_CURRENT_FORWARD)
                .withPeakReverseTorqueCurrent(ROLLER_PEAK_TORQUE_CURRENT_REVERSE))
        .withCurrentLimits(
            new CurrentLimitsConfigs().withSupplyCurrentLimit(ROLLER_SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(ROLLER_STATOR_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true));
    rollerMotor.getConfigurator().apply(rollerConfig);

    // Configure the CANcoder for the deploy
    var deployCancoderConfig = new CANcoderConfiguration();
    deployCancoderConfig.withMagnetSensor(
        new MagnetSensorConfigs().withMagnetOffset(DEPLOY_CANCODER_OFFSET)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(DEPLOY_DISCONTINUITY_POINT));
    deployCANcoder.getConfigurator().apply(deployCancoderConfig);

    // Configure the deploy motor
    var deployConfig = new TalonFXConfiguration();
    deployConfig.withMotorOutput(
        new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.Clockwise_Positive))
        .withFeedback(
            new FeedbackConfigs().withRotorToSensorRatio(DEPLOY_ROTOR_TO_SENSOR_RATIO)
                .withSensorToMechanismRatio(DEPLOY_SENSOR_TO_MECHANISM_RATIO)
                .withFusedCANcoder(deployCANcoder))
        .withSlot0(Slot0Configs.from(DEPLOY_SLOT_CONFIGS))
        .withMotionMagic(DEPLOY_MOTION_MAGIC_CONFIGS)
        .withCurrentLimits(
            new CurrentLimitsConfigs().withSupplyCurrentLimit(DEPLOY_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(DEPLOY_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true))
        .withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(DEPLOY_FORWARD_LIMIT)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(DEPLOY_REVERSE_LIMIT));
    deployMotor.getConfigurator().apply(deployConfig);

    // If the intake started in the retracted position, continue to hold it retracted. Otherwise, leave it neutral so it
    // stays deployed.
    if (isRetracted()) {
      // holdRetracted();
    }
  }

  /**
   * Command to run roller SysId routine in dynamic mode
   * 
   * @param direction The direction to run the roller motor for dynamic mode
   * @return The SysId output data for dynamic mode
   */
  public Command sysIdRollerDynamicCommand(Direction direction) {
    return rollerSysIdRoutine.dynamic(direction)
        .withName("SysId intake dynam " + direction)
        .finallyDo(this::stopIntaking);
  }

  /**
   * Command to run roller SysId routine in quasistatic mode
   * 
   * @param direction The direction to run the roller motor for quasistatic mode
   * @return The SysId output data for quasistatic mode
   */
  public Command sysIdRollerQuasistaticCommand(Direction direction) {
    return rollerSysIdRoutine.quasistatic(direction)
        .withName("SysId intake quasi " + direction)
        .finallyDo(this::stopIntaking);
  }

  /**
   * Command to run deploy SysId routine in dynamic mode
   * 
   * @param direction The direction to run the deploy motor for dynamic mode
   * @return The SysId output data for dynamic mode
   */
  public Command sysIdDeployDynamicCommand(Direction direction) {
    return deploySysIdRoutine.dynamic(direction)
        .withName("SysId deploy dynam " + direction)
        .finallyDo(this::stopDeploy);
  }

  /**
   * Command to run deploy SysId routine in quasistatic mode
   * 
   * @param direction The direction to run the deploy motor for quasistatic mode
   * @return The SysId output data for quasistatic mode
   */
  public Command sysIdDeployQuasistaticCommand(Direction direction) {
    return deploySysIdRoutine.quasistatic(direction)
        .withName("SysId deploy quasi " + direction)
        .finallyDo(this::stopDeploy);
  }

  /**
   * Runs the intake rollers to intake fuel
   */
  public void runIntake() {
    rollerMotor.setControl(rollerControl.withVelocity(ROLLER_INTAKE_VELOCITY));
  }

  /**
   * Reverses the intake rollers to eject fuel
   */
  public void reverseIntake() {
    rollerMotor.setControl(rollerControl.withVelocity(ROLLER_EJECT_VELOCITY));
  }

  /**
   * Deploys the intake
   */
  public void deploy() {
    deployMotor.setControl(deployControl.withPosition(DEPLOYED_POSITION));
  }

  /**
   * Retracts the intake
   */
  public void retract() {
    deployMotor.setControl(deployControl.withPosition(RETRACTED_POSITION));
  }

  /**
   * Applies some torque to hold the intake in the retracted position.
   */
  public void holdRetracted() {
    deployMotor.setControl(deployHoldControl);
  }

  /**
   * Stops the intake rollers
   */
  public void stopIntaking() {
    rollerMotor.stopMotor();
  }

  /**
   * Stops the deploy
   */
  public void stopDeploy() {
    deployMotor.stopMotor();
  }

  /**
   * Stops all intake motion
   */
  public void stop() {
    stopIntaking();
    stopDeploy();
  }

  /**
   * Checks if the intake is deployed
   * 
   * @return true if the intake is deployed, false otherwise
   */
  @Logged
  public boolean isDeployed() {
    BaseStatusSignal.refreshAll(deployPositionSignal, deployVelocitySignal);
    Angle deployPosition = BaseStatusSignal.getLatencyCompensatedValue(deployPositionSignal, deployVelocitySignal);
    return deployPosition.isNear(DEPLOYED_POSITION, DEPLOY_TOLERANCE);
  }

  /**
   * Checks if the intake is retracted
   * 
   * @return true if the intake is retracted, false otherwise
   */
  @Logged
  public boolean isRetracted() {
    BaseStatusSignal.refreshAll(deployPositionSignal, deployVelocitySignal);
    Angle deployPosition = BaseStatusSignal.getLatencyCompensatedValue(deployPositionSignal, deployVelocitySignal);
    return deployPosition.isNear(RETRACTED_POSITION, DEPLOY_TOLERANCE);
  }

  /**
   * Gets the current temperature of the deploy motor
   * 
   * @return temperature of the deploy motor
   */
  @Logged(name = "Deploy Motor Temp (C)")
  public double getDeployMotorTemp() {
    return deployTempSignal.refresh().getValue().in(Celsius);
  }

  /**
   * Gets the deploy motor temperature fault value
   * 
   * @return true if the deploy motor has a temperature fault, false otherwise
   */
  @Logged(name = "Deploy Motor Temp Fault")
  public boolean isDeployMotorDeviceTempFault() {
    return deployTempFaultSignal.refresh().getValue();
  }
}
