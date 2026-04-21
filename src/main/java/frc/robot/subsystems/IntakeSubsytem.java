//Copyright (c) FIRST and other WPILib contributors
//Open Source Software; can modify and/or share it under the terms of
//the WPILib BSD licensefile in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS;
import static frc.robot.Constants.IntakeConstants.CHANNEL_ID_DEPLOY_POTENTIOMETER;
import static frc.robot.Constants.IntakeConstants.DEPLOYED_POSITION;
import static frc.robot.Constants.IntakeConstants.DEPLOY_FORWARD_LIMIT;
import static frc.robot.Constants.IntakeConstants.DEPLOY_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.IntakeConstants.DEPLOY_PEAK_CURRENT_FORWARD;
import static frc.robot.Constants.IntakeConstants.DEPLOY_PEAK_CURRENT_REVERSE;
import static frc.robot.Constants.IntakeConstants.DEPLOY_REVERSE_LIMIT;
import static frc.robot.Constants.IntakeConstants.DEPLOY_SLOT_CONFIGS;
import static frc.robot.Constants.IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.IntakeConstants.DEPLOY_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.IntakeConstants.DEPLOY_TOLERANCE;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_DEPLOY_MOTOR;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_ROLLER_FOLLOWER;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_ROLLER_MOTOR;
import static frc.robot.Constants.IntakeConstants.POSE_DEPLOYED;
import static frc.robot.Constants.IntakeConstants.POSE_RETRACTED;
import static frc.robot.Constants.IntakeConstants.POTENTIOMETER_FULL_RANGE;
import static frc.robot.Constants.IntakeConstants.POTENTIOMETER_OFFSET;
import static frc.robot.Constants.IntakeConstants.RETRACTED_POSITION;
import static frc.robot.Constants.IntakeConstants.ROLLER_EJECT_VELOCITY;
import static frc.robot.Constants.IntakeConstants.ROLLER_INTAKE_SHOOTING_VELOCITY;
import static frc.robot.Constants.IntakeConstants.ROLLER_INTAKE_VELOCITY;
import static frc.robot.Constants.IntakeConstants.ROLLER_PEAK_TORQUE_CURRENT_FORWARD;
import static frc.robot.Constants.IntakeConstants.ROLLER_PEAK_TORQUE_CURRENT_REVERSE;
import static frc.robot.Constants.IntakeConstants.ROLLER_SLOT_CONFIGS;
import static frc.robot.Constants.IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.IntakeConstants.ROLLER_SUPPLY_CURRENT_LIMIT;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * Subsytem for the intake.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class IntakeSubsytem extends SubsystemBase {

  private final TalonFX rollerLeaderMotor = new TalonFX(DEVICE_ID_ROLLER_MOTOR, CANIVORE_BUS);
  private final TalonFX rollerFollowerMotor = new TalonFX(DEVICE_ID_ROLLER_FOLLOWER, CANIVORE_BUS);
  private final TalonFX deployMotor = new TalonFX(DEVICE_ID_DEPLOY_MOTOR, CANIVORE_BUS);

  // Motor request objects
  private final MotionMagicVoltage deployProfiledControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC deployTorqueControl = new TorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC rollerControl = new VelocityTorqueCurrentFOC(0.0);

  private final TorqueCurrentFOC rollerSysIdControl = new TorqueCurrentFOC(0.0);
  private final VoltageOut deploySysIdControl = new VoltageOut(0.0).withEnableFOC(true);

  private final StatusSignal<Angle> deployPositionSignal = deployMotor.getPosition(false);
  private final StatusSignal<AngularVelocity> deployVelocitySignal = deployMotor.getVelocity(false);

  @Logged(name = "Potentiometer")
  private final AnalogPotentiometer deploySensor = new AnalogPotentiometer(
      CHANNEL_ID_DEPLOY_POTENTIOMETER,
      POTENTIOMETER_FULL_RANGE.in(Rotations),
      POTENTIOMETER_OFFSET.in(Rotations));

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
          (amps) -> rollerLeaderMotor.setControl(rollerSysIdControl.withOutput(amps.in(Volts))),
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

  private Pose3d currentPose3d = POSE_RETRACTED;

  /**
   * Creates a new substyem for the intake
   */
  public IntakeSubsytem() {
    // Configure the roller motor
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig
        .withMotorOutput(
            new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive))
        .withSlot0(Slot0Configs.from(ROLLER_SLOT_CONFIGS))
        .withTorqueCurrent(
            new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(ROLLER_PEAK_TORQUE_CURRENT_FORWARD)
                .withPeakReverseTorqueCurrent(ROLLER_PEAK_TORQUE_CURRENT_REVERSE))
        .withCurrentLimits(
            new CurrentLimitsConfigs().withSupplyCurrentLimit(ROLLER_SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(ROLLER_STATOR_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true));
    rollerLeaderMotor.getConfigurator().apply(rollerConfig);
    rollerFollowerMotor.getConfigurator().apply(rollerConfig);

    // Max update frequency for leader for fast following
    rollerLeaderMotor.getTorqueCurrent(false).setUpdateFrequency(Hertz.of(1000));
    // Keep default update frequency for current signals for logging
    BaseStatusSignal.setUpdateFrequencyForAll(
        Hertz.of(100),
          rollerLeaderMotor.getVelocity(false),
          rollerLeaderMotor.getStatorCurrent(false),
          rollerLeaderMotor.getSupplyCurrent(false),
          rollerFollowerMotor.getStatorCurrent(false),
          rollerFollowerMotor.getSupplyCurrent(false));
    // Turn unused and follower signals down, but not off, for logging
    ParentDevice.optimizeBusUtilizationForAll(rollerLeaderMotor, rollerFollowerMotor);

    rollerFollowerMotor.setControl(new Follower(rollerLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    // Configure the deploy motor
    var deployConfig = new TalonFXConfiguration();
    deployConfig
        .withMotorOutput(
            new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive))
        .withSlot0(Slot0Configs.from(DEPLOY_SLOT_CONFIGS))
        .withMotionMagic(DEPLOY_MOTION_MAGIC_CONFIGS)
        .withTorqueCurrent(
            new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(DEPLOY_PEAK_CURRENT_FORWARD)
                .withPeakReverseTorqueCurrent(DEPLOY_PEAK_CURRENT_REVERSE))
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
    deployMotor.setPosition(getPotentiometerValue());

    // Keep default update frequency for important signals for logging
    BaseStatusSignal.setUpdateFrequencyForAll(
        Hertz.of(100),
          deployPositionSignal,
          deployVelocitySignal,
          deployMotor.getMotorVoltage(false),
          deployMotor.getStatorCurrent(false),
          deployMotor.getSupplyCurrent(false));
    // Turn unused signals down, but not off, for logging
    deployMotor.optimizeBusUtilization();
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

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(deployPositionSignal, deployVelocitySignal);
    // Update the intake pose for AdvantageScope
    Angle deployPosition = BaseStatusSignal.getLatencyCompensatedValue(deployPositionSignal, deployVelocitySignal);
    currentPose3d = POSE_RETRACTED.interpolate(POSE_DEPLOYED, deployPosition.div(DEPLOY_FORWARD_LIMIT).in(Value));
  }

  /**
   * Runs the intake rollers to intake fuel
   */
  public void runIntake() {
    rollerLeaderMotor.setControl(rollerControl.withVelocity(ROLLER_INTAKE_VELOCITY));
  }

  /**
   * Reverses the intake rollers to eject or unjam fuel
   */
  public void eject() {
    rollerLeaderMotor.setControl(rollerControl.withVelocity(ROLLER_EJECT_VELOCITY));
  }

  /**
   * Deploys the intake
   */
  public void deploy() {
    deployMotor.setControl(
        deployProfiledControl.withPosition(DEPLOYED_POSITION)
            .withLimitReverseMotion(getPotentiometerValue() >= DEPLOYED_POSITION.in(Rotations)));
    if (RobotBase.isSimulation()) {
      deployMotor.setPosition(DEPLOYED_POSITION);
    }
  }

  /**
   * Retracts the intake
   */
  public void retract() {
    deployMotor.setControl(
        deployProfiledControl.withPosition(RETRACTED_POSITION)
            .withLimitReverseMotion(getPotentiometerValue() <= RETRACTED_POSITION.in(Rotations)));
    if (RobotBase.isSimulation()) {
      deployMotor.setPosition(RETRACTED_POSITION);
    }
  }

  /**
   * Retracts the intake with a set current to help feed fuel into the feeder while shooting
   * 
   * @param current The current to apply to the deploy motor
   */
  public void retractForShooting(Current current) {
    deployMotor.setControl(deployTorqueControl.withOutput(current));
    if (RobotBase.isSimulation()) {
      deployMotor.setPosition(RETRACTED_POSITION);
    }
  }

  /**
   * Spins the intake rollers at a velocity to help feed fuel into the feeder while shooting
   */
  public void runIntakeForShooting() {
    rollerLeaderMotor.setControl(rollerControl.withVelocity(ROLLER_INTAKE_SHOOTING_VELOCITY));
  }

  /**
   * Stops the intake rollers
   */
  public void stopIntaking() {
    rollerLeaderMotor.stopMotor();
  }

  /**
   * Stops the deploy motor
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
    return (getDeployPosition().gte(DEPLOYED_POSITION.minus(DEPLOY_TOLERANCE))
        || getPotentiometerValue() >= DEPLOYED_POSITION.minus(DEPLOY_TOLERANCE).in(Rotations));
  }

  /**
   * Gets the current deploy position
   * 
   * @return current deploy position
   */
  public Angle getDeployPosition() {
    // Signals refreshed in periodic
    return BaseStatusSignal.getLatencyCompensatedValue(deployPositionSignal, deployVelocitySignal);
  }

  /**
   * Checks if the intake is retracted
   * 
   * @return true if the intake is retracted, false otherwise
   */
  @Logged
  public boolean isRetracted() {
    // Signals refreshed in periodic
    Angle deployPosition = BaseStatusSignal.getLatencyCompensatedValue(deployPositionSignal, deployVelocitySignal);
    return (deployPosition.lte(RETRACTED_POSITION.plus(DEPLOY_TOLERANCE))
        || getPotentiometerValue() <= RETRACTED_POSITION.plus(DEPLOY_TOLERANCE).in(Rotations));
  }

  /**
   * Gets the angular velocity of the deploy motor.
   * 
   * @return the angular velocity of the deploy motor
   */
  public AngularVelocity getDeployVelocity() {
    return deployVelocitySignal.refresh().getValue();
  }

  /**
   * Gets the pose of the intake for AdvantageScope.
   * <p>
   * <strong>This is not intended for use in robot code.</strong>
   * </p>
   * 
   * @return intake pose for AdvantageScope
   */
  @Logged
  public Pose3d getPose() {
    return currentPose3d;
  }

  /**
   * Gets the position of the intake from the potentiometer.
   * 
   * @return the position reading from the potentiometer
   */
  private double getPotentiometerValue() {
    return deploySensor.get();
  }

}
