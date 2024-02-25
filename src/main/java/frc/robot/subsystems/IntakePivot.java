// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.IntakePosition;

public class IntakePivot extends SubsystemBase {

  private final TalonFX LeftIntakePivot = new TalonFX(Constants.INTAKE_PIVOT_LEFT_ID);
  private final TalonFX RightIntakePivot = new TalonFX(Constants.INTAKE_PIVOT_RIGHT_ID);

  private final CurrentLimitsConfigs cl_cfg = new CurrentLimitsConfigs();

  private IntakePosition requested_position = IntakePosition.UP;

  private double minIntakePosition = Constants.INTAKE_UP_POS;

  /** Creates a new Intake. */
  public IntakePivot() {

    TalonFXConfiguration fx_cfg = new TalonFXConfiguration();

    cl_cfg.StatorCurrentLimit = Constants.INTAKE_CURRENT_LIMIT;
    cl_cfg.StatorCurrentLimitEnable = true;

    fx_cfg.CurrentLimits = cl_cfg;
    
    fx_cfg.Slot0.kP = Constants.INTAKE_kP;
    fx_cfg.Slot0.kI = Constants.INTAKE_kI;
    fx_cfg.Slot0.kD = Constants.INTAKE_kD;
    fx_cfg.Slot0.kA = 0.0;
    fx_cfg.Slot0.kG = 0.0;
    fx_cfg.Slot0.kS = 0.0;
    fx_cfg.Slot0.kV = Constants.INTAKE_kV;

    fx_cfg.MotionMagic.MotionMagicAcceleration = Constants.INTAKE_MOTIONMAGIC_ACC;
    fx_cfg.MotionMagic.MotionMagicCruiseVelocity = Constants.INTAKE_MOTIONMAGIC_CV;

    RightIntakePivot.getConfigurator().apply(fx_cfg);

    LeftIntakePivot.getConfigurator().apply(cl_cfg);

    LeftIntakePivot.setControl(new Follower(RightIntakePivot.getDeviceID(), true));

  }

  public void setIntakePosition(IntakePosition position)
  {
    this.requested_position = position;

    double raw_pos = 0;

    if(this.requested_position == IntakePosition.DOWN) {
      raw_pos = minIntakePosition + Constants.INTAKE_POS_DELTA;
    } else {
      raw_pos = minIntakePosition;
    }

    RightIntakePivot.setControl(new MotionMagicVoltage(raw_pos));
  }

  @Override
  public void periodic() {
    // Helps fix annoying issue where intake has to be up when powered on
    if(RightIntakePivot.getPosition().getValueAsDouble() < minIntakePosition) {
      minIntakePosition = RightIntakePivot.getPosition().getValueAsDouble();
    }
  }
}
