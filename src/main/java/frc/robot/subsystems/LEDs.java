// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class LEDs extends SubsystemBase {

	AddressableLED m_led = null;
	AddressableLEDBuffer m_ledBuffer = null;

	public LEDs() {
		// PWM port 9
		// Must be a PWM header, not MXP or DIO
		m_led = new AddressableLED(9);

		// Reuse buffer
		// Default to a length of 60, start empty output
		// Length is expensive to set, so only set it once, then just update data
		m_ledBuffer = new AddressableLEDBuffer(60);
		m_led.setLength(m_ledBuffer.getLength());

		// Set the data
		m_led.setData(m_ledBuffer);
		m_led.start();
	}

	public void setRed() {
		for (var i = 0; i < m_ledBuffer.getLength(); i++) {
			// Sets the specified LED to the RGB values for red
			m_ledBuffer.setRGB(i, 255, 0, 0);
		}

		m_led.setData(m_ledBuffer);
	}

	
}
