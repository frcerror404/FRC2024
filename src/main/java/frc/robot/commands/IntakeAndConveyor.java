// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.enums.IntakeConveyorSpeed;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAndConveyor extends InstantCommand {
  private final Intake m_intake;
  private final Conveyor m_conveyor;
  private final IntakeConveyorSpeed m_speed;

  public IntakeAndConveyor(Intake intake, Conveyor conveyor, IntakeConveyorSpeed speed) {
    m_conveyor = conveyor;
    m_intake = intake;
    m_speed = speed;

    addRequirements(intake, conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(m_speed) {
      case ON:
        m_conveyor.SetConveyorSpeed(0.5);
        m_intake.setIntakeSpeed(0.4);
        break;
      default:
        m_conveyor.SetConveyorSpeed(0.0);
        m_intake.setIntakeSpeed(0.0);
    }
  }
}
