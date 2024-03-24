// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Indexer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IndexerAndConveyor extends InstantCommand {
  private final Conveyor m_conveyor;
  private final Indexer m_indexer;
  private final double m_conveyorSpeed, m_indexerSpeed;

  public IndexerAndConveyor(Conveyor conveyor, Indexer indexer, double conveyorSpeed, double indexerSpeed) {
    m_conveyor = conveyor;
    m_indexer = indexer;
    m_conveyorSpeed = conveyorSpeed;
    m_indexerSpeed = indexerSpeed;


    addRequirements(m_indexer, m_conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_conveyor.SetConveyorSpeed(m_conveyorSpeed);
    m_indexer.setFeederSpeed(m_indexerSpeed);
  }
}
