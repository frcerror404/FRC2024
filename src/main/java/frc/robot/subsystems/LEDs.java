// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class LEDs extends SubsystemBase {
    private final AddressableLED m_leds = new AddressableLED(Constants.LED_PORT);
    private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(13);
    // private final BooleanSubscriber NoteStatus, TopWheelAtRPM, BottomWheelAtRPM;
    // private final DoubleSubscriber TopWheelTargetRPM;
    private boolean NoteStatus, TopWheelAtRPM, BottomWheelAtRPM;
    private double TopWheelTargetRPM;

    public LEDs() {
        // NetworkTableInstance nt = NetworkTableInstance.getDefault();
        
        // NetworkTable conveyor = nt.getTable("conveyor");
        // NetworkTable shooter = nt.getTable("shooter");

        // NoteStatus = conveyor.getBooleanTopic("NoteSensor").subscribe(false);
        // TopWheelAtRPM = shooter.getBooleanTopic("Top Wheel At Speed").subscribe(false);
        // BottomWheelAtRPM = shooter.getBooleanTopic("Bottom Wheel At Speed").subscribe(false);
        // TopWheelTargetRPM = shooter.getDoubleTopic("Bottom Wheel Target RPM").subscribe(0);

        m_leds.setLength(m_ledBuffer.getLength());
        m_leds.start();
    }

    public void setRobotStatus(boolean noteInConveyor, boolean topWheelAtRPM, boolean bottomWheelAtRPM, double targetRPM) {
        NoteStatus = noteInConveyor;
        TopWheelAtRPM = topWheelAtRPM;
        BottomWheelAtRPM = bottomWheelAtRPM;
        TopWheelTargetRPM = targetRPM;
    }

    @Override
    public void periodic() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }

        if(NoteStatus) {
            for(int i = 0; i < m_ledBuffer.getLength(); i++)
            {
                m_ledBuffer.setRGB(i, 75, 0, 130);
            }
        }

        if(TopWheelAtRPM && BottomWheelAtRPM && (TopWheelTargetRPM > 0)) {
            for(int i = 0; i < m_ledBuffer.getLength(); i++)
            {
                m_ledBuffer.setRGB(i, 0, 255, 0);
            }
        }

        m_leds.setData(m_ledBuffer);
    }
}
