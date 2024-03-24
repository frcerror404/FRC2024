// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class IntakeUntilNoteDetected extends Command {
  private final Conveyor m_conveyor;
  private final Intake m_intake;
  private final double m_conveyorSpeed, m_intakeSpeed;

  /** Creates a new IntakeUntilNoteDetected. */
  public IntakeUntilNoteDetected(Conveyor conveyor, Intake intake, double conveyorSpeed, double intakeSpeed) {
    m_conveyor = conveyor;
    m_intake = intake;
    m_conveyorSpeed = conveyorSpeed;
    m_intakeSpeed = intakeSpeed;
    
    addRequirements(m_conveyor, m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!m_conveyor.isNoteInConveyor()) {
      m_conveyor.SetConveyorSpeed(m_conveyorSpeed);
      m_intake.setIntakeSpeed(m_intakeSpeed);
    } else {
      m_conveyor.SetConveyorSpeed(0);
      m_intake.setIntakeSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyor.SetConveyorSpeed(0);
    m_intake.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_conveyor.isNoteInConveyor();
  }
}
