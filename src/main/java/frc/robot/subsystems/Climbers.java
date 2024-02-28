// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climbers extends SubsystemBase {

  private final TalonFX LeftClimberMotor = new TalonFX(Constants.CLIMBER_LEFT_ID);
  private final TalonFX RightClimberMotor = new TalonFX(Constants.CLIMBER_RIGHT_ID);

  private final DoublePublisher LClimberMotorTempOut;
  private final DoublePublisher RClimberMotorTempOut;

  CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
  TalonFXConfiguration config = new TalonFXConfiguration();

  /** Creates a new Climbers. */
  public Climbers() {

    NetworkTableInstance inst =  NetworkTableInstance.getDefault();
     NetworkTable tempTable = inst.getTable("temperature");

     LClimberMotorTempOut = tempTable.getDoubleTopic("Left Climber ").publish();
     RClimberMotorTempOut = tempTable.getDoubleTopic("Right Climber").publish();
     
    currentLimits.StatorCurrentLimit = Constants.CLIMBER_CURRENT_LIMIT;
    currentLimits.StatorCurrentLimitEnable = true;
    
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits = currentLimits;

    LeftClimberMotor.getConfigurator().apply(config);
    RightClimberMotor.getConfigurator().apply(config);


    //LeftClimberMotor.setControl(new Follower(RightClimberMotor.getDeviceID(), false));

  }

  public void SetClimberSpeed(double speed) {
    RightClimberMotor.setControl(new DutyCycleOut(speed));
    LeftClimberMotor.setControl(new DutyCycleOut(speed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    LClimberMotorTempOut.set(LeftClimberMotor.getDeviceTemp().getValueAsDouble());
    RClimberMotorTempOut.set(RightClimberMotor.getDeviceTemp().getValueAsDouble());
  }

  public void setBrakeMode(boolean enabled) {
    config.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    LeftClimberMotor.getConfigurator().apply(config);
    RightClimberMotor.getConfigurator().apply(config);
  }
}
