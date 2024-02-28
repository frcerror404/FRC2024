// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  CANSparkMax LeftIntakeMotor = new CANSparkMax(Constants.INTAKE_TOP_ID, MotorType.kBrushless);
  CANSparkMax RightIntakeMotor = new CANSparkMax(Constants.INTAKE_BOTTOM_ID, MotorType.kBrushless);

private final DoublePublisher LeftIntakeTempOut;
private final DoublePublisher RightIntakeTempOut;


  /** Creates a new Intake. */
  public Intake() {
    // Follow Inverted
    //RightIntakeMotor.follow(LeftIntakeMotor, true);
  
  NetworkTableInstance inst =  NetworkTableInstance.getDefault();
     NetworkTable tempTable = inst.getTable("temperature");

     RightIntakeTempOut = tempTable.getDoubleTopic("IntakeR").publish();
   LeftIntakeTempOut = tempTable.getDoubleTopic("IntakeL").publish();
    
  }

  public void setIntakeSpeed(double speed) {
    LeftIntakeMotor.set(speed);
    RightIntakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
RightIntakeTempOut.set(RightIntakeMotor.getMotorTemperature());
LeftIntakeTempOut.set(LeftIntakeMotor.getMotorTemperature());
    
  }
}
