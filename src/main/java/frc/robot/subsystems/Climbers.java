// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climbers extends SubsystemBase {

  private final TalonFX LeftClimberMotor = new TalonFX(Constants.CLIMBER_LEFT_ID);
  private final TalonFX RightClimberMotor = new TalonFX(Constants.CLIMBER_RIGHT_ID);

  /** Creates a new Climbers. */
  public Climbers() {

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimit = Constants.CLIMBER_CURRENT_LIMIT;
    currentLimits.StatorCurrentLimitEnable = true;

    LeftClimberMotor.getConfigurator().apply(currentLimits);
    RightClimberMotor.getConfigurator().apply(currentLimits);


    LeftClimberMotor.setControl(new Follower(RightClimberMotor.getDeviceID(), false));

  }

  public void SetClimberSpeed(double speed) {
    RightClimberMotor.setControl(new DutyCycleOut(speed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
