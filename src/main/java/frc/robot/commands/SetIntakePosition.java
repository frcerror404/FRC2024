// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.enums.IntakePosition;
import frc.robot.subsystems.IntakePivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetIntakePosition extends InstantCommand {
  private final IntakePivot m_IntakePivot;
  private final IntakePosition m_IntakePosition;

  public SetIntakePosition(IntakePivot intakePivot, IntakePosition position) {
    m_IntakePivot = intakePivot;
    m_IntakePosition = position;
    addRequirements(intakePivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakePivot.setIntakePosition(m_IntakePosition);
  }
}
