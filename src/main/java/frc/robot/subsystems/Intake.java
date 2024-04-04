// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  TalonFX LeftIntakeMotor = new TalonFX(Constants.INTAKE_TOP_ID);
  TalonFX RightIntakeMotor = new TalonFX(Constants.INTAKE_BOTTOM_ID);

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

    TalonFXConfiguration cc_limit = new TalonFXConfiguration();
    cc_limit.CurrentLimits.SupplyCurrentLimit = 30;
    cc_limit.CurrentLimits.SupplyCurrentLimitEnable = true;

    LeftIntakeMotor.getConfigurator().apply(cc_limit);
    RightIntakeMotor.getConfigurator().apply(cc_limit);
    
  }

  public void setIntakeSpeed(double speed) {

    if(speed < 0) {
      LeftIntakeMotor.setControl(new DutyCycleOut(speed));
      RightIntakeMotor.setControl(new DutyCycleOut(speed * 0.5));
    } else {
      LeftIntakeMotor.setControl(new DutyCycleOut(speed));
      RightIntakeMotor.setControl(new DutyCycleOut(speed));
    }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    RightIntakeTempOut.set(RightIntakeMotor.getDeviceTemp().getValueAsDouble());
    LeftIntakeTempOut.set(LeftIntakeMotor.getDeviceTemp().getValueAsDouble());
    
  }
}
