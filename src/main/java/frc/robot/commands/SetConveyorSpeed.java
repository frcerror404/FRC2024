// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Conveyor;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetConveyorSpeed extends InstantCommand {
  private final Conveyor m_Conveyor;
  private final double m_Speed;

  public SetConveyorSpeed(Conveyor conveyor, double speed) {
    m_Conveyor = conveyor;
    m_Speed = speed; 

    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_Conveyor.SetConveyorSpeed(m_Speed);
  }
}
