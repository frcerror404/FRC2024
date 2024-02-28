// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final TalonFX TopShooterMotor = new TalonFX(Constants.SHOOTER_TOP_ID);
  private final TalonFX BottomShooterMotor = new TalonFX(Constants.SHOOTER_BOTTOM_ID);

  private double targetRPM;

  private VoltageOut motorVoltageRequest = new VoltageOut(0);

  private final DoublePublisher TopWheelRPMOut;
  private final DoublePublisher BottomWheelRPMOut;
  private final DoublePublisher TargetRPMOut;
  //private final DoublePublisher RPMErrorOut;
  private final DoublePublisher TopMotorTemperatureOut;
  private final DoublePublisher BottomMotorTemperatureOut;



  public Shooter() {
    motorSetup();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable shooterTable = inst.getTable("shooter");
    NetworkTable temperatureTable = inst.getTable("temperature");
    
    TopWheelRPMOut = shooterTable.getDoubleTopic("Top Wheel RPM").publish();
    BottomWheelRPMOut = shooterTable.getDoubleTopic("Bottom Wheel RPM").publish();
    TargetRPMOut = shooterTable.getDoubleTopic("Target RPM").publish();
    TopMotorTemperatureOut = temperatureTable.getDoubleTopic("Shooter Top").publish();
    BottomMotorTemperatureOut = temperatureTable.getDoubleTopic("Shooter Bottom").publish();
  }

  @Override
  public void periodic() {
    TopWheelRPMOut.set(TopShooterMotor.getRotorVelocity().getValueAsDouble() * 60.0f);
    BottomWheelRPMOut.set(BottomShooterMotor.getRotorVelocity().getValueAsDouble() * 60.0f);
    TargetRPMOut.set(targetRPM);
    TopMotorTemperatureOut.set(TopShooterMotor.getDeviceTemp().getValueAsDouble());
    BottomMotorTemperatureOut.set(BottomShooterMotor.getDeviceTemp().getValueAsDouble());
  }

  private void motorSetup() {
    TalonFXConfiguration motorA_cfg = new TalonFXConfiguration();
    motorA_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    TopShooterMotor.getConfigurator().apply(motorA_cfg);
    
    BottomShooterMotor.setControl(new Follower(TopShooterMotor.getDeviceID(), true));
  }



  public void setShooterSpeed(double speed) {
    motorVoltageRequest.withOutput(12.0 * speed);
    TopShooterMotor.setControl(motorVoltageRequest.withOutput(12.0 * speed));
  }
}
