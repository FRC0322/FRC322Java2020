/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dashboard extends SubsystemBase {
	private ShuffleboardTab driverShuffleboardTab;
	private ShuffleboardTab autonomousShuffleboardTab;
	private ShuffleboardTab debuggerShuffleboardTab;
	/**
	 * Creates a new Dashboard.
	 */
	public Dashboard() {
		driverShuffleboardTab = Shuffleboard.getTab("Driver");
		autonomousShuffleboardTab = Shuffleboard.getTab("Autonomous");
		debuggerShuffleboardTab = Shuffleboard.getTab("Debugger");
	}

	public enum Tab {
		kDriver(0),
		kAutonomous(1),
		kDebugger(2);

		private static final Map<Integer, Tab> MY_MAP = new HashMap<Integer, Tab>();

		static {
			for (Tab tab : values()) {
				MY_MAP.put(tab.getValue(), tab);
			}
		}

		private int value;

		private Tab(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}

		public static Tab getByValue(int value) {
			return MY_MAP.get(value);
		}

		public String toString() {
			return name();
		}
	}

	public void setTab(Tab tab) {
		if (tab == Tab.kDriver)
			Shuffleboard.selectTab("Driver");
		else if (tab == Tab.kAutonomous)
			Shuffleboard.selectTab("Autonomous");
		else if (tab == Tab.kDebugger)
			Shuffleboard.selectTab("Debugger");
	}

	public ShuffleboardTab getDriverTab() {
		return driverShuffleboardTab;
	}

	public ShuffleboardTab getAutonomousTab() {
		return autonomousShuffleboardTab;
	}

	public ShuffleboardTab getDebuggerTab() {
		return debuggerShuffleboardTab;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
