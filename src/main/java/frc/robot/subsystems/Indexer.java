// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  TalonFX ShooterFeederMotor = new TalonFX(Constants.SHOOTER_FEEDER_ID);
  
private final DoublePublisher ShooterFeederMotorTempOut, IndexerRPMout;
  
  /** Creates a new Conveyor. */
  public Indexer() {

 NetworkTableInstance inst =  NetworkTableInstance.getDefault();
    NetworkTable tempTable = inst.getTable("temperature");
    NetworkTable convTable = inst.getTable("conveyor");

    ShooterFeederMotorTempOut = tempTable.getDoubleTopic("Feeder Motor").publish();
    IndexerRPMout = convTable.getDoubleTopic("Indexer RPM").publish();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ShooterFeederMotorTempOut.set(ShooterFeederMotor.getDeviceTemp().getValueAsDouble());
    IndexerRPMout.set(ShooterFeederMotor.getVelocity().getValueAsDouble() * 60);
  }

  public void setFeederSpeed(double speed)
  {
    speed = -speed;

    ShooterFeederMotor.setControl(new DutyCycleOut(speed));
  }
}
