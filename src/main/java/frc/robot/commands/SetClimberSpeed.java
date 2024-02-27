// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climbers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetClimberSpeed extends InstantCommand {
  private final Climbers m_climbers;
  private final double m_speed;

  public SetClimberSpeed(Climbers climbers, double speed) {
    m_climbers = climbers;
    m_speed = speed;

    addRequirements(climbers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climbers.SetClimberSpeed(m_speed);
  }
}
