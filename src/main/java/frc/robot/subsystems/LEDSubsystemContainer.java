package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microsecond;
import static edu.wpi.first.wpilibj.LEDPattern.kOff;
import static frc.robot.Constants.LEDConstants.BACK_LED_STRIP_LENGTH;
import static frc.robot.Constants.LEDConstants.DEVICE_ID_LEDS;
import static frc.robot.Constants.LEDConstants.INTAKE_HIGH_LED_STRIP_LENGTH;
import static frc.robot.Constants.LEDConstants.INTAKE_LOW_LED_STRIP_LENGTH;
import static frc.robot.Constants.LEDConstants.LEFT_LED_STRIP_LENGTH;
import static frc.robot.Constants.LEDConstants.TOTAL_LEDS;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

/**
 * Container for LED subsystems for controlling the LEDs
 */
public class LEDSubsystemContainer {

  private final AddressableLED leds = new AddressableLED(DEVICE_ID_LEDS);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(TOTAL_LEDS);
  private final AddressableLEDBufferView intakeHighBuffer;
  private final AddressableLEDBufferView intakeLowBuffer;
  private final AddressableLEDBufferView leftBuffer;
  private final AddressableLEDBufferView backBuffer;
  private final AddressableLEDBufferView intakeHighLeftBuffer;
  private final AddressableLEDBufferView intakeHighRightBuffer;
  private final AddressableLEDBufferView intakeLowLeftBuffer;
  private final AddressableLEDBufferView intakeLowRightBuffer;

  private final IntakeLEDSubsystem intakeLEDSubsystem = new IntakeLEDSubsystem();
  private final RobotLEDSubsystem robotLEDSubsystem = new RobotLEDSubsystem();

  public interface LEDSubsystem extends Subsystem {
    void off();

    void runPatternOnAll(LEDPattern pattern);

    default Command runPatternOnAllCommand(LEDPattern pattern) {
      return run(() -> runPatternOnAll(pattern)).finallyDo(this::off);
    }
  }

  /**
   * Creates a new LEDSubsystemContainer
   */
  public LEDSubsystemContainer() {
    leds.setLength(TOTAL_LEDS);
    leds.setData(ledBuffer);

    backBuffer = new AddressableLEDBufferView(ledBuffer, 0, BACK_LED_STRIP_LENGTH - 1);
    leftBuffer = new AddressableLEDBufferView(
        ledBuffer,
        BACK_LED_STRIP_LENGTH,
        BACK_LED_STRIP_LENGTH + LEFT_LED_STRIP_LENGTH - 1);
    intakeLowBuffer = new AddressableLEDBufferView(
        ledBuffer,
        BACK_LED_STRIP_LENGTH + LEFT_LED_STRIP_LENGTH,
        BACK_LED_STRIP_LENGTH + LEFT_LED_STRIP_LENGTH + INTAKE_LOW_LED_STRIP_LENGTH - 1);
    intakeHighBuffer = new AddressableLEDBufferView(
        ledBuffer,
        BACK_LED_STRIP_LENGTH + LEFT_LED_STRIP_LENGTH + INTAKE_LOW_LED_STRIP_LENGTH,
        BACK_LED_STRIP_LENGTH + LEFT_LED_STRIP_LENGTH + INTAKE_LOW_LED_STRIP_LENGTH + INTAKE_HIGH_LED_STRIP_LENGTH - 1)
        .reversed();

    intakeHighLeftBuffer = new AddressableLEDBufferView(intakeHighBuffer, 0, (INTAKE_HIGH_LED_STRIP_LENGTH - 1) / 2);
    intakeHighRightBuffer = new AddressableLEDBufferView(
        intakeHighBuffer,
        INTAKE_HIGH_LED_STRIP_LENGTH / 2,
        INTAKE_HIGH_LED_STRIP_LENGTH - 1);
    intakeLowLeftBuffer = new AddressableLEDBufferView(intakeLowBuffer, 0, (INTAKE_LOW_LED_STRIP_LENGTH - 1) / 2);
    intakeLowRightBuffer = new AddressableLEDBufferView(
        intakeLowBuffer,
        INTAKE_LOW_LED_STRIP_LENGTH / 2,
        INTAKE_LOW_LED_STRIP_LENGTH - 1);

    leds.start();
    new LEDSuperSubsystem(); // TODO consider a better way to schedule this periodic update
  }

  private class LEDSuperSubsystem extends SubsystemBase {
    @Override
    public void periodic() {
      leds.setData(ledBuffer);
    }
  }

  public IntakeLEDSubsystem getIntakeLEDSubsystem() {
    return intakeLEDSubsystem;
  }

  public RobotLEDSubsystem getRobotLEDSubsystem() {
    return robotLEDSubsystem;
  }

  /**
   * Creates an LEDPattern that makes an animated candy cane effect (alternating colors)
   * 
   * @param color1 The first color
   * @param color2 The second color
   * @param period The length of time before swapping the colors
   */
  public static LEDPattern candyCane(Color color1, Color color2, Time period) {
    var periodMicros = (long) period.in(Microsecond);
    return (reader, writer) -> {
      var isOdd = (RobotController.getTime() / periodMicros) % 2 == 1;
      for (int led = 0; led < reader.getLength(); led++) {
        if (isOdd) {
          writer.setLED(led, led % 2 == 0 ? color1 : color2);
        } else {
          writer.setLED(led, led % 2 == 0 ? color2 : color1);
        }
      }
    };
  }

  /**
   * Creates an LEDPattern that lights up the LEDs in segments. Useful for indicating ready state, for example.
   *
   * @param color the color of the segments when lit
   * @param segmentValues array of boolean suppliers. The strip will be split into segments one segment for each
   *          element
   *          of the array.
   */
  public static LEDPattern ledSegments(Color color, BooleanSupplier... segmentValues) {
    return (reader, writer) -> {
      final int ledsPerStatus = reader.getLength() / segmentValues.length;
      int ledIndex = 0;
      for (int segmentId = 0; segmentId < segmentValues.length; segmentId++) {
        for (; ledIndex < (ledsPerStatus * (segmentId + 1)); ledIndex++) {
          writer.setLED(ledIndex, segmentValues[segmentId].getAsBoolean() ? color : Color.kBlack);
        }
      }
      // In case the LEDs can't be perfectly divided into segments, turn off any remaining LEDs
      for (; ledIndex < reader.getLength(); ledIndex++) {
        writer.setLED(ledIndex, Color.kBlack);
      }
    };
  }

  public class IntakeLEDSubsystem extends SubsystemBase implements LEDSubsystem {

    /**
     * Applies the pattern to the intake high LED strip. This will only happen once. If running an animation, this
     * method must be called continuously to update the led states.
     * 
     * @param pattern Pattern to set on the intake high LED strip
     */
    public void runPatternOnIntakeHigh(LEDPattern pattern) {
      pattern.applyTo(intakeHighBuffer);
    }

    /**
     * Applies the pattern to the left side of the intake high LED strip. This will only happen once. If running an
     * animation, this
     * method must be called continuously to update the led states.
     * 
     * @param pattern Pattern to set on the left side of the intake high LED strip
     */
    public void runPatternOnIntakeHighLeft(LEDPattern pattern) {
      pattern.applyTo(intakeHighLeftBuffer);
    }

    /**
     * Applies the pattern to the right side of the intake high LED strip. This will only happen once. If running an
     * animation, this
     * method must be called continuously to update the led states.
     * 
     * @param pattern Pattern to set on the right side of the intake high LED strip
     */
    public void runPatternOnIntakeHighRight(LEDPattern pattern) {
      pattern.applyTo(intakeHighRightBuffer);
    }

    /**
     * Applies the pattern to the intake low LED strip. This will only happen once. If running an animation, this
     * method must be called continuously to update the led states.
     * 
     * @param pattern Pattern to set on the intake low LED strip
     */
    public void runPatternOnIntakeLow(LEDPattern pattern) {
      pattern.applyTo(intakeLowBuffer);
    }

    /**
     * Applies the pattern to the left side of the intake low LED strip. This will only happen once. If running an
     * animation, this
     * method must be called continuously to update the led states.
     * 
     * @param pattern Pattern to set on the left side of the intake low LED strip
     */
    public void runPatternOnIntakeLowLeft(LEDPattern pattern) {
      pattern.applyTo(intakeLowLeftBuffer);
    }

    /**
     * Applies the pattern to the right side of the intake low LED strip. This will only happen once. If running an
     * animation, this
     * method must be called continuously to update the led states.
     * 
     * @param pattern Pattern to set on the right side of the intake low LED strip
     */
    public void runPatternOnIntakeLowRight(LEDPattern pattern) {
      pattern.applyTo(intakeLowRightBuffer);
    }

    /**
     * Applies the pattern to both intake LED strip. This will only happen once. If running an animation, this
     * method must be called continuously to update the led states.
     * 
     * @param pattern Pattern to set on the intake LED strips
     */
    @Override
    public void runPatternOnAll(LEDPattern pattern) {
      runPatternOnIntakeHigh(pattern);
      runPatternOnIntakeLow(pattern);
    }

    /**
     * Gets the length of the LED strip on the high part of the intake.
     * 
     * @return length of the strips
     */
    public int getIntakeStripLength() {
      return INTAKE_HIGH_LED_STRIP_LENGTH;
    }

    /**
     * Turns off the LEDs
     */
    public void off() {
      runPatternOnIntakeHigh(kOff);
      runPatternOnIntakeLow(kOff);
    }

    /**
     * Turns off the LEDs on the intake high strip
     */
    public void offIntakeHigh() {
      runPatternOnIntakeHigh(kOff);
    }

    /**
     * Turns off the LEDs on the intake low strip
     */
    public void offIntakeLow() {
      runPatternOnIntakeLow(kOff);
    }
  }

  public class RobotLEDSubsystem extends SubsystemBase implements LEDSubsystem {

    /**
     * Applies the pattern to the left LED strip. This will only happen once. If running an animation, this
     * method must be called continuously to update the led states.
     * 
     * @param pattern Pattern to set on the left LED strip
     */
    public void runPatternOnLeft(LEDPattern pattern) {
      pattern.applyTo(leftBuffer);
    }

    /**
     * Applies the pattern to the back LED strip. This will only happen once. If running an animation, this
     * method must be called continuously to update the led states.
     * 
     * @param pattern Pattern to set on the back LED strip
     */
    public void runPatternOnBack(LEDPattern pattern) {
      pattern.applyTo(backBuffer);
    }

    @Override
    public void runPatternOnAll(LEDPattern pattern) {
      runPatternOnLeft(pattern);
      runPatternOnBack(pattern);
    }

    /**
     * Gets the length of the LED strip on the high part of the intake.
     * 
     * @return length of the strips
     */
    public int getIntakeHighStripLength() {
      return INTAKE_HIGH_LED_STRIP_LENGTH;
    }

    /**
     * Gets the length of the LED strip on the low part of the intake.
     * 
     * @return length of the strips
     */
    public int getIntakeLowStripLength() {
      return INTAKE_LOW_LED_STRIP_LENGTH;
    }

    /**
     * Gets the length of the LED strip on the left side of the robot.
     * 
     * @return length of the strips
     */
    public int getLeftStripLength() {
      return LEFT_LED_STRIP_LENGTH;
    }

    /**
     * Gets the length of the LED strip on the back of the robot.
     * 
     * @return length of the strips
     */
    public int getBackStripLength() {
      return BACK_LED_STRIP_LENGTH;
    }

    /**
     * Turns off the LEDs
     */
    public void off() {
      runPatternOnLeft(kOff);
      runPatternOnBack(kOff);
    }

    /**
     * Turns off the LEDs on the left strip
     */
    public void offLeft() {
      runPatternOnLeft(kOff);
    }

    /**
     * Turns off the LEDs on the back strip
     */
    public void offBack() {
      runPatternOnBack(kOff);
    }
  }
}