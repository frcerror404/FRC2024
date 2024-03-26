// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterAngle extends SubsystemBase {

  private final TalonFX angleMotor = new TalonFX(Constants.ANGLE_MOTOR_ID);
  private final CANcoder angleEncoder = new CANcoder(Constants.ANGLE_ENCODER_ID);

  private final CurrentLimitsConfigs cl_cfg = new CurrentLimitsConfigs();

  private final DoublePublisher motorEncoderOut;
  private final DoublePublisher motorTemperatureOut;
  private final DoublePublisher shooterAnglePositionOut;
  private final DoublePublisher shooterAngleTargetOut;
  private final DoublePublisher shooterAngleTargetRawOut;
  private final DoublePublisher motorCurrentOut;

  private double shooterAngleTarget = 65.0, shooterAngleTargetRaw = 0.0;

  private final MotionMagicVoltage motorCommand = new MotionMagicVoltage(0);


  /** Creates a new ShooterAngle. */
  public ShooterAngle() {
    motorSetup();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable shooterTable = inst.getTable("shooter");
    NetworkTable tempTable = inst.getTable("temperature");
    
    motorEncoderOut = shooterTable.getDoubleTopic("Angle Motor Encoder").publish();
    motorCurrentOut = shooterTable.getDoubleTopic("Angle Motor Current").publish();
    shooterAnglePositionOut = shooterTable.getDoubleTopic("Shooter Angle Position").publish();
    shooterAngleTargetOut = shooterTable.getDoubleTopic("Nice Angle Target").publish();
    shooterAngleTargetRawOut = shooterTable.getDoubleTopic("Raw Angle Target").publish();
    
    motorTemperatureOut = tempTable.getDoubleTopic("Shooter Angle").publish();
  }

  private void motorSetup() 
  {
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cc_cfg.MagnetSensor.MagnetOffset = Constants.ANGLE_ENCODER_OFFSET;
    angleEncoder.getConfigurator().apply(cc_cfg);

    TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
    fx_cfg.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    fx_cfg.Feedback.SensorToMechanismRatio = 1.0;
    fx_cfg.Feedback.RotorToSensorRatio = Constants.ANGLE_GEAR_RATIO;

    cl_cfg.StatorCurrentLimit = Constants.ANGLE_MOTOR_CURRENT_LIMIT;
    cl_cfg.StatorCurrentLimitEnable = true;

    fx_cfg.CurrentLimits = cl_cfg;
    
    fx_cfg.Slot0.kP = Constants.ANGLE_PID_kP;
    fx_cfg.Slot0.kI = Constants.ANGLE_PID_kI;
    fx_cfg.Slot0.kD = Constants.ANGLE_PID_kD;
    fx_cfg.Slot0.kA = 0.0;
    fx_cfg.Slot0.kG = 0.0;
    fx_cfg.Slot0.kS = 0.0;
    fx_cfg.Slot0.kV = 0.0;

    fx_cfg.MotionMagic.MotionMagicAcceleration = Constants.ANGLE_MOTIONMAGIC_ACC;
    fx_cfg.MotionMagic.MotionMagicCruiseVelocity = Constants.ANGLE_MOTIONMAGIC_CV;

    angleMotor.getConfigurator().apply(fx_cfg);

  }

  @Override
  public void periodic() 
  {
    motorEncoderOut.set(getShooterAngle());
    motorTemperatureOut.set(angleMotor.getDeviceTemp().getValueAsDouble());
    shooterAnglePositionOut.set(angleMotor.getPosition().getValueAsDouble());
    shooterAngleTargetOut.set(shooterAngleTarget);
    shooterAngleTargetRawOut.set(shooterAngleTargetRaw);
    motorCurrentOut.set(angleMotor.getStatorCurrent().getValueAsDouble());

    // if(getShooterAngle() < 70.0 && shooterAngleTarget == Constants.LOCATION_HOME) 
    // {
    //   angleMotor.getConfigurator().apply(cl_cfg.withStatorCurrentLimit(Constants.ANGLE_MOTOR_CURRENT_LIMIT_HOME));
    // } else {
    //   angleMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.ANGLE_MOTOR_CURRENT_LIMIT)); 
    // }
  }

  public double getShooterAngle()
  {
    var rawAngle = angleEncoder.getAbsolutePosition();
    double angle = (rawAngle.getValueAsDouble() * 360.0f) + Constants.SHOOTER_ANGLE_ZERO_OFFSET;

    return angle;
  }

  public void setShooterAngle(double angleDegrees)
  {
    shooterAngleTarget = angleDegrees;
    shooterAngleTargetRaw = (angleDegrees - Constants.SHOOTER_ANGLE_ZERO_OFFSET) / 360.0f;

    // Phoenix Tuner X liked kP = 150, kI = 15, kD = 0 for Motion Magic
    // Also, positions 0.00 - 0.12 was the range of motion for the shooter
    // Unsure of whether to put the desired angle (Fused CANCoder) or the motor position?

    angleMotor.setControl(motorCommand.withPosition(shooterAngleTargetRaw));
  }

  public void zeroMotorEncoder() 
  {
    angleMotor.setPosition(0);
  }

  public boolean isShooterAtAngle(double toleranceDegrees) {
    return Math.abs(getShooterAngle() - shooterAngleTarget) < toleranceDegrees;
  }

  public boolean isShooterAtAngle() {
    return isShooterAtAngle(Constants.SHOOTER_ANGLE_DEFAULT_TOLERANCE);
  }
}
