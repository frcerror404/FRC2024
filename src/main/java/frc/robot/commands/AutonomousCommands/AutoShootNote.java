// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;

public class AutoShootNote extends Command {
  private final Conveyor m_conveyor;
  private final Indexer m_indexer;
  private final ShooterAngle m_shooterAngle;
  private final Shooter m_shooter;
  private final double m_topRPM, m_bottomRPM, m_targetAngle;
  
  private final double m_timeoutS;
  private double m_startingTimestamp, m_noteOutTimestamp = Double.NaN;


  /** Creates a new AutoShootNote. */
  public AutoShootNote(Conveyor conveyor, Indexer indexer, ShooterAngle shooterAngle, Shooter shooter, double targetAngle, double topRPM, double bottomRPM, double timeoutS) {
    m_conveyor = conveyor;
    m_indexer = indexer;
    m_shooterAngle = shooterAngle;
    m_shooter = shooter;
    m_timeoutS = timeoutS;
    m_targetAngle = targetAngle;
    m_topRPM = topRPM;
    m_bottomRPM = bottomRPM;

    addRequirements(m_conveyor, m_indexer, m_shooterAngle, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startingTimestamp = Timer.getFPGATimestamp();

    m_shooter.setShooterRPM(m_topRPM, m_bottomRPM);
    m_shooterAngle.setShooterAngle(m_targetAngle);

    System.out.println("AutoShootNote Initialized");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_shooter.isBottomWheelAtTargetVelocity() && m_shooter.isTopWheelAtTargetVelocity() && m_shooterAngle.isShooterAtAngle())
    {
      m_conveyor.SetConveyorSpeed(.5);
      m_indexer.setFeederSpeed(1.0);
    }

    if(!m_conveyor.isNoteInConveyor()){
      m_noteOutTimestamp = Timer.getFPGATimestamp();
    } else {
      m_noteOutTimestamp = Double.NaN;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterAngle.setShooterAngle(Constants.LOCATION_HOME);
    m_shooter.setShooterSpeed(0.0);
    m_conveyor.SetConveyorSpeed(0.0);
    m_indexer.setFeederSpeed(0.0);

    System.out.println("AutoShootNote Ended");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean ranOutOfTime = (Timer.getFPGATimestamp() - m_startingTimestamp) > m_timeoutS;

    if(ranOutOfTime) {
      System.out.println("AutoShoot - Ran Out Of Time");
    }
    boolean noteWasShot = false;
    
    if(m_noteOutTimestamp != Double.NaN) {
      noteWasShot = (Timer.getFPGATimestamp() - m_noteOutTimestamp) > Constants.AUTO_SHOOT_NOTE_OUT_DELAY;
    } 

    if(noteWasShot) {
      System.out.println("AutoShoot - Note Shot");
    }
    
    return ranOutOfTime || noteWasShot;
  }
}
