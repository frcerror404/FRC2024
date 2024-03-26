// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.IntakePosition;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;

public class AutoIntake extends Command {
  private final Intake m_intake;
  private final Conveyor m_Conveyor;
  private final IntakePivot m_IntakePivot;
  private final double m_timeoutS;
  private double m_startingTimestamp;

  /** Creates a new AutoIntake. */
  public AutoIntake(Intake intake, Conveyor conveyor, IntakePivot ip, double timeoutS) {
    m_intake = intake;
    m_Conveyor = conveyor;
    m_IntakePivot = ip;
    m_timeoutS = timeoutS;
    
    addRequirements(m_Conveyor, m_IntakePivot, m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startingTimestamp = Timer.getFPGATimestamp();

    if(!m_Conveyor.isNoteInConveyor()){
      m_Conveyor.SetConveyorSpeed(.5);
      m_intake.setIntakeSpeed(.5);
      m_IntakePivot.setIntakePosition(IntakePosition.DOWN);
    }

    System.out.println("AutoIntake Initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Conveyor.SetConveyorSpeed(0.0);
    m_intake.setIntakeSpeed(0.0);
    m_IntakePivot.setIntakePosition(IntakePosition.UP);

    System.out.println("AutoIntake Ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean ranOutOfTime = (Timer.getFPGATimestamp() - m_startingTimestamp) > m_timeoutS;
    return ranOutOfTime | m_Conveyor.isNoteInConveyor();
  }
}
