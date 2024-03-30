// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Conveyor;

public class AutoStopConveyor extends InstantCommand {
  private final Conveyor m_conveyor;
  private final double m_conveyorSpeed;
  /** Creates a new AutoStopConveyor. */
  public AutoStopConveyor(Conveyor conveyor, double speed) {
    m_conveyor = conveyor;
    m_conveyorSpeed = speed;

    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_conveyor.SetConveyorSpeedAutoStop(m_conveyorSpeed);
  }
}
