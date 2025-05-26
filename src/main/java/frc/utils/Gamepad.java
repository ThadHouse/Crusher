// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

/**
 * Handle input from Gamepad controllers connected to the Driver Station.
 *
 * <p>This class handles Gamepad input that comes from the Driver Station. Each time a value is
 * requested the most recent value is returned. There is a single class instance for each controller
 * and the mapping of ports to hardware buttons depends on the code in the Driver Station.
 *
 * <p>Only first party controllers from  are guaranteed to have the correct mapping, and
 * only through the official NI DS. Sim is not guaranteed to have the same mapping, as well as any
 * 3rd party controllers.
 */
public class Gamepad extends GenericHID implements Sendable {
  /** Represents a digital button on a Gamepad. */
  public enum Button {
    /** South button. */
    kSouth(1),
    /** East button. */
    kEast(2),
    /** West button. */
    kWest(3),
    /** North button. */
    kNorth(4),
    /** Back button. */
    kBack(5),
    /** Guide button. */
    kGuide(6),
    /** Start button. */
    kStart(7),
    /** Left Stick button. */
    kLeftStick(8),
    /** Right Stick button. */
    kRightStick(9),
    /** Left Shoulder button. */
    kLeftShoulder(10),
    /** Right Shoulder button. */
    kRightShoulder(11),
    /** DPad Up button. */
    kDpadUp(12),
    /** DPad Down button. */
    kDpadDown(13),
    /** DPad Left button. */
    kDpadLeft(14),
    /** DPad Right button. */
    kDpadRight(15),
    /** Misc 1 button. */
    kMisc1(16),
    /** Right Paddle 1 button. */
    kRightPaddle1(17),
    /** Left Paddle 1 button. */
    kLeftPaddle1(18),
    /** Right Paddle 2 button. */
    kRightPaddle2(19),
    /** Left Paddle 2 button. */
    kLeftPaddle2(20),
    /** Touchpad button. */
    kTouchpad(21),
    /** Misc 2 button. */
    kMisc2(22),
    /** Misc 3 button. */
    kMisc3(23),
    /** Misc 4 button. */
    kMisc4(24),
    /** Misc 5 button. */
    kMisc5(25),
    /** Misc 6 button. */
    kMisc6(26);

    /** Button value. */
    public final int value;

    Button(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the button, matching the relevant methods. This is done by
     * stripping the leading `k`, and appending `Button`.
     *
     * <p>Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the button.
     */
    @Override
    public String toString() {
      // Remove leading `k`
      return this.name().substring(1) + "Button";
    }
  }

  /** Represents an axis on an Gamepad. */
  public enum Axis {
    /** Left X axis. */
    kLeftX(0),
    /** Right X axis. */
    kRightX(2),
    /** Left Y axis. */
    kLeftY(1),
    /** Right Y axis. */
    kRightY(3),
    /** Left trigger. */
    kLeftTrigger(4),
    /** Right trigger. */
    kRightTrigger(5);

    /** Axis value. */
    public final int value;

    Axis(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the axis, matching the relevant methods. This is done by
     * stripping the leading `k`, and appending `Axis` if the name ends with `Trigger`.
     *
     * <p>Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the axis.
     */
    @Override
    public String toString() {
      var name = this.name().substring(1); // Remove leading `k`
      if (name.endsWith("Trigger")) {
        return name + "Axis";
      }
      return name;
    }
  }

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into (0-5).
   */
  public Gamepad(final int port) {
    super(port);
    HAL.reportUsage("HID", port, "Gamepad");
  }

  /**
   * Get the X axis value of left side of the controller. Right is positive.
   *
   * @return The axis value.
   */
  public double getLeftX() {
    return getRawAxis(Axis.kLeftX.value);
  }

  /**
   * Get the X axis value of right side of the controller. Right is positive.
   *
   * @return The axis value.
   */
  public double getRightX() {
    return getRawAxis(Axis.kRightX.value);
  }

  /**
   * Get the Y axis value of left side of the controller. Back is positive.
   *
   * @return The axis value.
   */
  public double getLeftY() {
    return getRawAxis(Axis.kLeftY.value);
  }

  /**
   * Get the Y axis value of right side of the controller. Back is positive.
   *
   * @return The axis value.
   */
  public double getRightY() {
    return getRawAxis(Axis.kRightY.value);
  }

  /**
   * Get the Left Trigger axis value of the controller. Note that this axis is bound to the
   * range of [0, 1] as opposed to the usual [-1, 1].
   *
   * @return The axis value.
   */
  public double getLeftTriggerAxis() {
    return getRawAxis(Axis.kLeftTrigger.value);
  }

  /**
   * Constructs an event instance around the axis value of the Left Trigger. The returned trigger
   * will be true when the axis value is greater than {@code threshold}.
   *
   * @param threshold the minimum axis value for the returned {@link BooleanEvent} to be true. This
   *     value should be in the range [0, 1] where 0 is the unpressed state of the axis.
   * @param loop the event loop instance to attach the event to.
   * @return an event instance that is true when the Left Trigger's axis exceeds the provided
   *     threshold, attached to the given event loop
   */
  public BooleanEvent leftTrigger(double threshold, EventLoop loop) {
    return axisGreaterThan(Axis.kLeftTrigger.value, threshold, loop);
  }

  /**
   * Constructs an event instance around the axis value of the Left Trigger. The returned trigger
   * will be true when the axis value is greater than 0.5.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance that is true when the Left Trigger's axis exceeds the provided
   *     threshold, attached to the given event loop
   */
  public BooleanEvent leftTrigger(EventLoop loop) {
    return leftTrigger(0.5, loop);
  }

  /**
   * Get the Right Trigger axis value of the controller. Note that this axis is bound to the
   * range of [0, 1] as opposed to the usual [-1, 1].
   *
   * @return The axis value.
   */
  public double getRightTriggerAxis() {
    return getRawAxis(Axis.kRightTrigger.value);
  }

  /**
   * Constructs an event instance around the axis value of the Right Trigger. The returned trigger
   * will be true when the axis value is greater than {@code threshold}.
   *
   * @param threshold the minimum axis value for the returned {@link BooleanEvent} to be true. This
   *     value should be in the range [0, 1] where 0 is the unpressed state of the axis.
   * @param loop the event loop instance to attach the event to.
   * @return an event instance that is true when the Right Trigger's axis exceeds the provided
   *     threshold, attached to the given event loop
   */
  public BooleanEvent rightTrigger(double threshold, EventLoop loop) {
    return axisGreaterThan(Axis.kRightTrigger.value, threshold, loop);
  }

  /**
   * Constructs an event instance around the axis value of the Right Trigger. The returned trigger
   * will be true when the axis value is greater than 0.5.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance that is true when the Right Trigger's axis exceeds the provided
   *     threshold, attached to the given event loop
   */
  public BooleanEvent rightTrigger(EventLoop loop) {
    return rightTrigger(0.5, loop);
  }

  /**
   * Read the value of the South button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getSouthButton() {
    return getRawButton(Button.kSouth.value);
  }

  /**
   * Whether the South button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getSouthButtonPressed() {
    return getRawButtonPressed(Button.kSouth.value);
  }

  /**
   * Whether the South button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getSouthButtonReleased() {
    return getRawButtonReleased(Button.kSouth.value);
  }

  /**
   * Constructs an event instance around the South button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the South button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent south(EventLoop loop) {
    return button(Button.kSouth.value, loop);
  }

  /**
   * Read the value of the East button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getEastButton() {
    return getRawButton(Button.kEast.value);
  }

  /**
   * Whether the East button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getEastButtonPressed() {
    return getRawButtonPressed(Button.kEast.value);
  }

  /**
   * Whether the East button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getEastButtonReleased() {
    return getRawButtonReleased(Button.kEast.value);
  }

  /**
   * Constructs an event instance around the East button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the East button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent east(EventLoop loop) {
    return button(Button.kEast.value, loop);
  }

  /**
   * Read the value of the West button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getWestButton() {
    return getRawButton(Button.kWest.value);
  }

  /**
   * Whether the West button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getWestButtonPressed() {
    return getRawButtonPressed(Button.kWest.value);
  }

  /**
   * Whether the West button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getWestButtonReleased() {
    return getRawButtonReleased(Button.kWest.value);
  }

  /**
   * Constructs an event instance around the West button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the West button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent west(EventLoop loop) {
    return button(Button.kWest.value, loop);
  }

  /**
   * Read the value of the North button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getNorthButton() {
    return getRawButton(Button.kNorth.value);
  }

  /**
   * Whether the North button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getNorthButtonPressed() {
    return getRawButtonPressed(Button.kNorth.value);
  }

  /**
   * Whether the North button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getNorthButtonReleased() {
    return getRawButtonReleased(Button.kNorth.value);
  }

  /**
   * Constructs an event instance around the North button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the North button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent north(EventLoop loop) {
    return button(Button.kNorth.value, loop);
  }

  /**
   * Read the value of the Back button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getBackButton() {
    return getRawButton(Button.kBack.value);
  }

  /**
   * Whether the Back button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getBackButtonPressed() {
    return getRawButtonPressed(Button.kBack.value);
  }

  /**
   * Whether the Back button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getBackButtonReleased() {
    return getRawButtonReleased(Button.kBack.value);
  }

  /**
   * Constructs an event instance around the Back button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Back button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent back(EventLoop loop) {
    return button(Button.kBack.value, loop);
  }

  /**
   * Read the value of the Guide button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getGuideButton() {
    return getRawButton(Button.kGuide.value);
  }

  /**
   * Whether the Guide button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getGuideButtonPressed() {
    return getRawButtonPressed(Button.kGuide.value);
  }

  /**
   * Whether the Guide button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getGuideButtonReleased() {
    return getRawButtonReleased(Button.kGuide.value);
  }

  /**
   * Constructs an event instance around the Guide button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Guide button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent guide(EventLoop loop) {
    return button(Button.kGuide.value, loop);
  }

  /**
   * Read the value of the Start button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getStartButton() {
    return getRawButton(Button.kStart.value);
  }

  /**
   * Whether the Start button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getStartButtonPressed() {
    return getRawButtonPressed(Button.kStart.value);
  }

  /**
   * Whether the Start button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getStartButtonReleased() {
    return getRawButtonReleased(Button.kStart.value);
  }

  /**
   * Constructs an event instance around the Start button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Start button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent start(EventLoop loop) {
    return button(Button.kStart.value, loop);
  }

  /**
   * Read the value of the Left Stick button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getLeftStickButton() {
    return getRawButton(Button.kLeftStick.value);
  }

  /**
   * Whether the Left Stick button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getLeftStickButtonPressed() {
    return getRawButtonPressed(Button.kLeftStick.value);
  }

  /**
   * Whether the Left Stick button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getLeftStickButtonReleased() {
    return getRawButtonReleased(Button.kLeftStick.value);
  }

  /**
   * Constructs an event instance around the Left Stick button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Left Stick button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent leftStick(EventLoop loop) {
    return button(Button.kLeftStick.value, loop);
  }

  /**
   * Read the value of the Right Stick button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getRightStickButton() {
    return getRawButton(Button.kRightStick.value);
  }

  /**
   * Whether the Right Stick button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getRightStickButtonPressed() {
    return getRawButtonPressed(Button.kRightStick.value);
  }

  /**
   * Whether the Right Stick button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getRightStickButtonReleased() {
    return getRawButtonReleased(Button.kRightStick.value);
  }

  /**
   * Constructs an event instance around the Right Stick button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Right Stick button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent rightStick(EventLoop loop) {
    return button(Button.kRightStick.value, loop);
  }

  /**
   * Read the value of the Left Shoulder button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getLeftShoulderButton() {
    return getRawButton(Button.kLeftShoulder.value);
  }

  /**
   * Whether the Left Shoulder button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getLeftShoulderButtonPressed() {
    return getRawButtonPressed(Button.kLeftShoulder.value);
  }

  /**
   * Whether the Left Shoulder button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getLeftShoulderButtonReleased() {
    return getRawButtonReleased(Button.kLeftShoulder.value);
  }

  /**
   * Constructs an event instance around the Left Shoulder button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Left Shoulder button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent leftShoulder(EventLoop loop) {
    return button(Button.kLeftShoulder.value, loop);
  }

  /**
   * Read the value of the Right Shoulder button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getRightShoulderButton() {
    return getRawButton(Button.kRightShoulder.value);
  }

  /**
   * Whether the Right Shoulder button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getRightShoulderButtonPressed() {
    return getRawButtonPressed(Button.kRightShoulder.value);
  }

  /**
   * Whether the Right Shoulder button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getRightShoulderButtonReleased() {
    return getRawButtonReleased(Button.kRightShoulder.value);
  }

  /**
   * Constructs an event instance around the Right Shoulder button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Right Shoulder button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent rightShoulder(EventLoop loop) {
    return button(Button.kRightShoulder.value, loop);
  }

  /**
   * Read the value of the DPad Up button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getDpadUpButton() {
    return getRawButton(Button.kDpadUp.value);
  }

  /**
   * Whether the DPad Up button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getDpadUpButtonPressed() {
    return getRawButtonPressed(Button.kDpadUp.value);
  }

  /**
   * Whether the DPad Up button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getDpadUpButtonReleased() {
    return getRawButtonReleased(Button.kDpadUp.value);
  }

  /**
   * Constructs an event instance around the DPad Up button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the DPad Up button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent dpadUp(EventLoop loop) {
    return button(Button.kDpadUp.value, loop);
  }

  /**
   * Read the value of the DPad Down button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getDpadDownButton() {
    return getRawButton(Button.kDpadDown.value);
  }

  /**
   * Whether the DPad Down button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getDpadDownButtonPressed() {
    return getRawButtonPressed(Button.kDpadDown.value);
  }

  /**
   * Whether the DPad Down button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getDpadDownButtonReleased() {
    return getRawButtonReleased(Button.kDpadDown.value);
  }

  /**
   * Constructs an event instance around the DPad Down button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the DPad Down button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent dpadDown(EventLoop loop) {
    return button(Button.kDpadDown.value, loop);
  }

  /**
   * Read the value of the DPad Left button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getDpadLeftButton() {
    return getRawButton(Button.kDpadLeft.value);
  }

  /**
   * Whether the DPad Left button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getDpadLeftButtonPressed() {
    return getRawButtonPressed(Button.kDpadLeft.value);
  }

  /**
   * Whether the DPad Left button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getDpadLeftButtonReleased() {
    return getRawButtonReleased(Button.kDpadLeft.value);
  }

  /**
   * Constructs an event instance around the DPad Left button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the DPad Left button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent dpadLeft(EventLoop loop) {
    return button(Button.kDpadLeft.value, loop);
  }

  /**
   * Read the value of the DPad Right button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getDpadRightButton() {
    return getRawButton(Button.kDpadRight.value);
  }

  /**
   * Whether the DPad Right button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getDpadRightButtonPressed() {
    return getRawButtonPressed(Button.kDpadRight.value);
  }

  /**
   * Whether the DPad Right button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getDpadRightButtonReleased() {
    return getRawButtonReleased(Button.kDpadRight.value);
  }

  /**
   * Constructs an event instance around the DPad Right button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the DPad Right button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent dpadRight(EventLoop loop) {
    return button(Button.kDpadRight.value, loop);
  }

  /**
   * Read the value of the Misc 1 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getMisc1Button() {
    return getRawButton(Button.kMisc1.value);
  }

  /**
   * Whether the Misc 1 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getMisc1ButtonPressed() {
    return getRawButtonPressed(Button.kMisc1.value);
  }

  /**
   * Whether the Misc 1 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getMisc1ButtonReleased() {
    return getRawButtonReleased(Button.kMisc1.value);
  }

  /**
   * Constructs an event instance around the Misc 1 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Misc 1 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent misc1(EventLoop loop) {
    return button(Button.kMisc1.value, loop);
  }

  /**
   * Read the value of the Right Paddle 1 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getRightPaddle1Button() {
    return getRawButton(Button.kRightPaddle1.value);
  }

  /**
   * Whether the Right Paddle 1 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getRightPaddle1ButtonPressed() {
    return getRawButtonPressed(Button.kRightPaddle1.value);
  }

  /**
   * Whether the Right Paddle 1 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getRightPaddle1ButtonReleased() {
    return getRawButtonReleased(Button.kRightPaddle1.value);
  }

  /**
   * Constructs an event instance around the Right Paddle 1 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Right Paddle 1 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent rightPaddle1(EventLoop loop) {
    return button(Button.kRightPaddle1.value, loop);
  }

  /**
   * Read the value of the Left Paddle 1 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getLeftPaddle1Button() {
    return getRawButton(Button.kLeftPaddle1.value);
  }

  /**
   * Whether the Left Paddle 1 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getLeftPaddle1ButtonPressed() {
    return getRawButtonPressed(Button.kLeftPaddle1.value);
  }

  /**
   * Whether the Left Paddle 1 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getLeftPaddle1ButtonReleased() {
    return getRawButtonReleased(Button.kLeftPaddle1.value);
  }

  /**
   * Constructs an event instance around the Left Paddle 1 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Left Paddle 1 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent leftPaddle1(EventLoop loop) {
    return button(Button.kLeftPaddle1.value, loop);
  }

  /**
   * Read the value of the Right Paddle 2 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getRightPaddle2Button() {
    return getRawButton(Button.kRightPaddle2.value);
  }

  /**
   * Whether the Right Paddle 2 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getRightPaddle2ButtonPressed() {
    return getRawButtonPressed(Button.kRightPaddle2.value);
  }

  /**
   * Whether the Right Paddle 2 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getRightPaddle2ButtonReleased() {
    return getRawButtonReleased(Button.kRightPaddle2.value);
  }

  /**
   * Constructs an event instance around the Right Paddle 2 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Right Paddle 2 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent rightPaddle2(EventLoop loop) {
    return button(Button.kRightPaddle2.value, loop);
  }

  /**
   * Read the value of the Left Paddle 2 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getLeftPaddle2Button() {
    return getRawButton(Button.kLeftPaddle2.value);
  }

  /**
   * Whether the Left Paddle 2 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getLeftPaddle2ButtonPressed() {
    return getRawButtonPressed(Button.kLeftPaddle2.value);
  }

  /**
   * Whether the Left Paddle 2 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getLeftPaddle2ButtonReleased() {
    return getRawButtonReleased(Button.kLeftPaddle2.value);
  }

  /**
   * Constructs an event instance around the Left Paddle 2 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Left Paddle 2 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent leftPaddle2(EventLoop loop) {
    return button(Button.kLeftPaddle2.value, loop);
  }

  /**
   * Read the value of the Touchpad button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getTouchpadButton() {
    return getRawButton(Button.kTouchpad.value);
  }

  /**
   * Whether the Touchpad button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getTouchpadButtonPressed() {
    return getRawButtonPressed(Button.kTouchpad.value);
  }

  /**
   * Whether the Touchpad button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getTouchpadButtonReleased() {
    return getRawButtonReleased(Button.kTouchpad.value);
  }

  /**
   * Constructs an event instance around the Touchpad button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Touchpad button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent touchpad(EventLoop loop) {
    return button(Button.kTouchpad.value, loop);
  }

  /**
   * Read the value of the Misc 2 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getMisc2Button() {
    return getRawButton(Button.kMisc2.value);
  }

  /**
   * Whether the Misc 2 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getMisc2ButtonPressed() {
    return getRawButtonPressed(Button.kMisc2.value);
  }

  /**
   * Whether the Misc 2 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getMisc2ButtonReleased() {
    return getRawButtonReleased(Button.kMisc2.value);
  }

  /**
   * Constructs an event instance around the Misc 2 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Misc 2 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent misc2(EventLoop loop) {
    return button(Button.kMisc2.value, loop);
  }

  /**
   * Read the value of the Misc 3 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getMisc3Button() {
    return getRawButton(Button.kMisc3.value);
  }

  /**
   * Whether the Misc 3 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getMisc3ButtonPressed() {
    return getRawButtonPressed(Button.kMisc3.value);
  }

  /**
   * Whether the Misc 3 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getMisc3ButtonReleased() {
    return getRawButtonReleased(Button.kMisc3.value);
  }

  /**
   * Constructs an event instance around the Misc 3 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Misc 3 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent misc3(EventLoop loop) {
    return button(Button.kMisc3.value, loop);
  }

  /**
   * Read the value of the Misc 4 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getMisc4Button() {
    return getRawButton(Button.kMisc4.value);
  }

  /**
   * Whether the Misc 4 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getMisc4ButtonPressed() {
    return getRawButtonPressed(Button.kMisc4.value);
  }

  /**
   * Whether the Misc 4 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getMisc4ButtonReleased() {
    return getRawButtonReleased(Button.kMisc4.value);
  }

  /**
   * Constructs an event instance around the Misc 4 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Misc 4 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent misc4(EventLoop loop) {
    return button(Button.kMisc4.value, loop);
  }

  /**
   * Read the value of the Misc 5 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getMisc5Button() {
    return getRawButton(Button.kMisc5.value);
  }

  /**
   * Whether the Misc 5 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getMisc5ButtonPressed() {
    return getRawButtonPressed(Button.kMisc5.value);
  }

  /**
   * Whether the Misc 5 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getMisc5ButtonReleased() {
    return getRawButtonReleased(Button.kMisc5.value);
  }

  /**
   * Constructs an event instance around the Misc 5 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Misc 5 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent misc5(EventLoop loop) {
    return button(Button.kMisc5.value, loop);
  }

  /**
   * Read the value of the Misc 6 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getMisc6Button() {
    return getRawButton(Button.kMisc6.value);
  }

  /**
   * Whether the Misc 6 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getMisc6ButtonPressed() {
    return getRawButtonPressed(Button.kMisc6.value);
  }

  /**
   * Whether the Misc 6 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getMisc6ButtonReleased() {
    return getRawButtonReleased(Button.kMisc6.value);
  }

  /**
   * Constructs an event instance around the Misc 6 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Misc 6 button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent misc6(EventLoop loop) {
    return button(Button.kMisc6.value, loop);
  }


  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("HID");
    builder.publishConstString("ControllerType", "Gamepad");
    // builder.addDoubleProperty("LeftTrigger", this::getLeftTriggerAxis, null);
    // builder.addDoubleProperty("RightTrigger", this::getRightTriggerAxis, null);
    // builder.addDoubleProperty("LeftX", this::getLeftX, null);
    // builder.addDoubleProperty("RightX", this::getRightX, null);
    // builder.addDoubleProperty("LeftY", this::getLeftY, null);
    // builder.addDoubleProperty("RightY", this::getRightY, null);
    // builder.addBooleanProperty("South", this::getSouthButton, null);
    // builder.addBooleanProperty("East", this::getEastButton, null);
    // builder.addBooleanProperty("West", this::getWestButton, null);
    // builder.addBooleanProperty("North", this::getNorthButton, null);
    // builder.addBooleanProperty("Back", this::getBackButton, null);
    // builder.addBooleanProperty("Guide", this::getGuideButton, null);
    // builder.addBooleanProperty("Start", this::getStartButton, null);
    // builder.addBooleanProperty("LeftStick", this::getLeftStickButton, null);
    // builder.addBooleanProperty("RightStick", this::getRightStickButton, null);
    // builder.addBooleanProperty("LeftShoulder", this::getLeftShoulderButton, null);
    // builder.addBooleanProperty("RightShoulder", this::getRightShoulderButton, null);
    // builder.addBooleanProperty("DpadUp", this::getDpadUpButton, null);
    // builder.addBooleanProperty("DpadDown", this::getDpadDownButton, null);
    // builder.addBooleanProperty("DpadLeft", this::getDpadLeftButton, null);
    // builder.addBooleanProperty("DpadRight", this::getDpadRightButton, null);
    // builder.addBooleanProperty("Misc1", this::getMisc1Button, null);
    // builder.addBooleanProperty("RightPaddle1", this::getRightPaddle1Button, null);
    // builder.addBooleanProperty("LeftPaddle1", this::getLeftPaddle1Button, null);
    // builder.addBooleanProperty("RightPaddle2", this::getRightPaddle2Button, null);
    // builder.addBooleanProperty("LeftPaddle2", this::getLeftPaddle2Button, null);
    // builder.addBooleanProperty("Touchpad", this::getTouchpadButton, null);
    // builder.addBooleanProperty("Misc2", this::getMisc2Button, null);
    // builder.addBooleanProperty("Misc3", this::getMisc3Button, null);
    // builder.addBooleanProperty("Misc4", this::getMisc4Button, null);
    // builder.addBooleanProperty("Misc5", this::getMisc5Button, null);
    // builder.addBooleanProperty("Misc6", this::getMisc6Button, null);
  }
}