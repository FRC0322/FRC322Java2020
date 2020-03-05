package frc.robot.utilities;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * Handle input from Logitech RumblePad 2 controllers connected to the Driver Station.
 *
 * <p>This class handles RumblePad input that comes from the Driver Station. Each time a value is
 * requested the most recent value is returned. There is a single class instance for each controller
 * and the mapping of ports to hardware buttons depends on the code in the Driver Station.
 */
public class RumblePad2 extends GenericHID {
	/**
	 * Represents a digital button on a RumblePad2.
	 */
	private enum Button {
		k1(1),
		k2(2),
		k3(3),
		k4(4),
		kBumperLeft1(5),
		kBumperRight1(6),
		kBumperLeft2(7),
		kBumperRight2(8),
		kBack(9),
		kStart(10),
		kStickLeft(11),
		kStickRight(12);

		//@SuppressWarnings("MemberName")
		private int value;

		Button(int value) {
			this.value = value;
		}
	}

	/**
	 * Construct an instance of a joystick. The joystick index is the USB port on the drivers
	 * station.
	 *
	 * @param port The port on the Driver Station that the joystick is plugged into.
	 */
	public RumblePad2(final int port) {
		super(port);

		// HAL.report(tResourceType.kResourceType_F310Controller, port);
		HAL.report(tResourceType.kResourceType_Joystick, port);
	}

	/**
	 * Get the X axis value of the controller.
	 *
	 * @param hand Side of controller whose value should be returned.
	 * @return The X axis value of the controller.
	 */
	@Override
	public double getX(Hand hand) {
		if (hand.equals(Hand.kLeft)) {
			return getRawAxis(0);
		} else {
			return getRawAxis(2);
		}
	}

	/**
	 * Get the Y axis value of the controller.
	 *
	 * @param hand Side of controller whose value should be returned.
	 * @return The Y axis value of the controller.
	 */
	@Override
	public double getY(Hand hand) {
		if (hand.equals(Hand.kLeft)) {
			return getRawAxis(1);
		} else {
			return getRawAxis(3);
		}
	}

	/**
	 * Read the value of the upper bumper button on the controller.
	 *
	 * @param hand Side of controller whose value should be returned.
	 * @return The state of the button.
	 */
	public boolean getUpperBumper(Hand hand) {
		if (hand.equals(Hand.kLeft)) {
			return getRawButton(Button.kBumperLeft1.value);
		} else {
			return getRawButton(Button.kBumperRight1.value);
		}
	}

	/**
	 * Whether the upper bumper was pressed since the last check.
	 *
	 * @param hand Side of controller whose value should be returned.
	 * @return Whether the button was pressed since the last check.
	 */
	public boolean getUpperBumperPressed(Hand hand) {
		if (hand == Hand.kLeft) {
			return getRawButtonPressed(Button.kBumperLeft1.value);
		} else {
			return getRawButtonPressed(Button.kBumperRight1.value);
		}
	}

	/**
	 * Whether the upper bumper was released since the last check.
	 *
	 * @param hand Side of controller whose value should be returned.
	 * @return Whether the button was released since the last check.
	 */
	public boolean getUpperBumperReleased(Hand hand) {
		if (hand == Hand.kLeft) {
			return getRawButtonReleased(Button.kBumperLeft1.value);
		} else {
			return getRawButtonReleased(Button.kBumperRight1.value);
		}
	}

	/**
	 * Read the value of the lower bumper button on the controller.
	 *
	 * @param hand Side of controller whose value should be returned.
	 * @return The state of the button.
	 */
	public boolean getLowerBumper(Hand hand) {
		if (hand.equals(Hand.kLeft)) {
			return getRawButton(Button.kBumperLeft2.value);
		} else {
			return getRawButton(Button.kBumperRight2.value);
		}
	}

	/**
	 * Whether the lower bumper was pressed since the last check.
	 *
	 * @param hand Side of controller whose value should be returned.
	 * @return Whether the button was pressed since the last check.
	 */
	public boolean getLowerBumperPressed(Hand hand) {
		if (hand == Hand.kLeft) {
			return getRawButtonPressed(Button.kBumperLeft2.value);
		} else {
			return getRawButtonPressed(Button.kBumperRight2.value);
		}
	}

	/**
	 * Whether the lower bumper was released since the last check.
	 *
	 * @param hand Side of controller whose value should be returned.
	 * @return Whether the button was released since the last check.
	 */
	public boolean getLowerBumperReleased(Hand hand) {
		if (hand == Hand.kLeft) {
			return getRawButtonReleased(Button.kBumperLeft2.value);
		} else {
			return getRawButtonReleased(Button.kBumperRight2.value);
		}
	}

	/**
	 * Read the value of the stick button on the controller.
	 *
	 * @param hand Side of controller whose value should be returned.
	 * @return The state of the button.
	 */
	public boolean getStickButton(Hand hand) {
		if (hand.equals(Hand.kLeft)) {
			return getRawButton(Button.kStickLeft.value);
		} else {
			return getRawButton(Button.kStickRight.value);
		}
	}

	/**
	 * Whether the stick button was pressed since the last check.
	 *
	 * @param hand Side of controller whose value should be returned.
	 * @return Whether the button was pressed since the last check.
	 */
	public boolean getStickButtonPressed(Hand hand) {
		if (hand == Hand.kLeft) {
			return getRawButtonPressed(Button.kStickLeft.value);
		} else {
			return getRawButtonPressed(Button.kStickRight.value);
		}
	}

	/**
	 * Whether the stick button was released since the last check.
	 *
	 * @param hand Side of controller whose value should be returned.
	 * @return Whether the button was released since the last check.
	 */
	public boolean getStickButtonReleased(Hand hand) {
		if (hand == Hand.kLeft) {
			return getRawButtonReleased(Button.kStickLeft.value);
		} else {
			return getRawButtonReleased(Button.kStickRight.value);
		}
	}

	/**
	 * Read the value of the "1" button on the controller.
	 *
	 * @return The state of the button.
	 */
	public boolean getButton1() {
		return getRawButton(Button.k1.value);
	}

	/**
	 * Whether the "1" button was pressed since the last check.
	 *
	 * @return Whether the button was pressed since the last check.
	 */
	public boolean getButton1Pressed() {
		return getRawButtonPressed(Button.k1.value);
	}

	/**
	 * Whether the "1" button was released since the last check.
	 *
	 * @return Whether the button was released since the last check.
	 */
	public boolean getButton1Released() {
		return getRawButtonReleased(Button.k1.value);
	}

	/**
	 * Read the value of the "2" button on the controller.
	 *
	 * @return The state of the button.
	 */
	public boolean getButton2() {
		return getRawButton(Button.k2.value);
	}

	/**
	 * Whether the "2" button was pressed since the last check.
	 *
	 * @return Whether the button was pressed since the last check.
	 */
	public boolean getButton2Pressed() {
		return getRawButtonPressed(Button.k2.value);
	}

	/**
	 * Whether the "2" button was released since the last check.
	 *
	 * @return Whether the button was released since the last check.
	 */
	public boolean getButton2Released() {
		return getRawButtonReleased(Button.k2.value);
	}

	/**
	 * Read the value of the "3" button on the controller.
	 *
	 * @return The state of the button.
	 */
	public boolean getButton3() {
		return getRawButton(Button.k3.value);
	}

	/**
	 * Whether the "3" button was pressed since the last check.
	 *
	 * @return Whether the button was pressed since the last check.
	 */
	public boolean getButton3Pressed() {
		return getRawButtonPressed(Button.k3.value);
	}

	/**
	 * Whether the "3" button was released since the last check.
	 *
	 * @return Whether the button was released since the last check.
	 */
	public boolean getButton3Released() {
		return getRawButtonReleased(Button.k3.value);
	}

	/**
	 * Read the value of the "4" button on the controller.
	 *
	 * @return The state of the button.
	 */
	public boolean getButton4() {
		return getRawButton(Button.k4.value);
	}

	/**
	 * Whether the "4" button was pressed since the last check.
	 *
	 * @return Whether the button was pressed since the last check.
	 */
	public boolean getButton4Pressed() {
		return getRawButtonPressed(Button.k4.value);
	}

	/**
	 * Whether the "4" button was released since the last check.
	 *
	 * @return Whether the button was released since the last check.
	 */
	public boolean getButton4Released() {
		return getRawButtonReleased(Button.k4.value);
	}

	/**
	 * Read the value of the back button on the controller.
	 *
	 * @return The state of the button.
	 */
	public boolean getBackButton() {
		return getRawButton(Button.kBack.value);
	}

	/**
	 * Whether the back button was pressed since the last check.
	 *
	 * @return Whether the button was pressed since the last check.
	 */
	public boolean getBackButtonPressed() {
		return getRawButtonPressed(Button.kBack.value);
	}

	/**
	 * Whether the back button was released since the last check.
	 *
	 * @return Whether the button was released since the last check.
	 */
	public boolean getBackButtonReleased() {
		return getRawButtonReleased(Button.kBack.value);
	}

	/**
	 * Read the value of the start button on the controller.
	 *
	 * @return The state of the button.
	 */
	public boolean getStartButton() {
		return getRawButton(Button.kStart.value);
	}

	/**
	 * Whether the start button was pressed since the last check.
	 *
	 * @return Whether the button was pressed since the last check.
	 */
	public boolean getStartButtonPressed() {
		return getRawButtonPressed(Button.kStart.value);
	}

	/**
	 * Whether the start button was released since the last check.
	 *
	 * @return Whether the button was released since the last check.
	 */
	public boolean getStartButtonReleased() {
		return getRawButtonReleased(Button.kStart.value);
	}
}
