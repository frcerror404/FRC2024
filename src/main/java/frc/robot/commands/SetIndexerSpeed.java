// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Conveyor;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetIndexerSpeed extends InstantCommand {
  private final Conveyor m_conveyor;
  private final double m_speed;

  public SetIndexerSpeed(Conveyor conveyor, double speed) {
    m_conveyor = conveyor;
    m_speed = speed;

    addRequirements(m_conveyor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_conveyor.setFeederSpeed(m_speed);
  }
}
