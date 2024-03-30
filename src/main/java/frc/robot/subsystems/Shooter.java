// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.networktables.BooleanPublisher;
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
  private final DoublePublisher TopWheelRPMTargetOut;
  private final DoublePublisher BottomWheelRPMTargetOut;
  private final BooleanPublisher TopWheelAtTargetRPMOut;
  private final BooleanPublisher BottomWheelAtTargetRPMOut;
  private final DoublePublisher BottomWheelCurrentOut;
  private final DoublePublisher TopWheelCurrentOut;
  //private final DoublePublisher RPMErrorOut;
  private final DoublePublisher TopMotorTemperatureOut;
  private final DoublePublisher BottomMotorTemperatureOut;

  private double TopRPMTarget = 0, BottomRPMTarget = 0;

  private final MotionMagicVelocityVoltage topMM = new MotionMagicVelocityVoltage(0);
  private final MotionMagicVelocityVoltage bottomMM = new MotionMagicVelocityVoltage(0);



  public Shooter() {
    motorSetup();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable shooterTable = inst.getTable("shooter");
    NetworkTable temperatureTable = inst.getTable("temperature");
    
    TopWheelRPMOut = shooterTable.getDoubleTopic("Top Wheel RPM").publish();
    BottomWheelRPMOut = shooterTable.getDoubleTopic("Bottom Wheel RPM").publish();
    
    TopWheelRPMTargetOut = shooterTable.getDoubleTopic("Top Wheel Target RPM").publish();
    BottomWheelRPMTargetOut = shooterTable.getDoubleTopic("Bottom Wheel Target RPM").publish();
    TopWheelAtTargetRPMOut = shooterTable.getBooleanTopic("Top Wheel At Speed").publish();
    BottomWheelAtTargetRPMOut = shooterTable.getBooleanTopic("Bottom Wheel At Speed").publish();

    BottomWheelCurrentOut = shooterTable.getDoubleTopic("Bottom Current").publish();
    TopWheelCurrentOut = shooterTable.getDoubleTopic("Top Current").publish();



    TopMotorTemperatureOut = temperatureTable.getDoubleTopic("Shooter Top").publish();
    BottomMotorTemperatureOut = temperatureTable.getDoubleTopic("Shooter Bottom").publish();
  }

  @Override
  public void periodic() {
    TopWheelRPMOut.set(TopShooterMotor.getRotorVelocity().getValueAsDouble() * 60.0f);
    TopMotorTemperatureOut.set(TopShooterMotor.getDeviceTemp().getValueAsDouble());
    TopWheelRPMTargetOut.set(TopRPMTarget);
    TopWheelAtTargetRPMOut.set(isTopWheelAtTargetVelocity());
    TopWheelCurrentOut.set(TopShooterMotor.getSupplyCurrent().getValueAsDouble());
    
    BottomWheelRPMOut.set(BottomShooterMotor.getRotorVelocity().getValueAsDouble() * 60.0f);
    BottomMotorTemperatureOut.set(BottomShooterMotor.getDeviceTemp().getValueAsDouble());
    BottomWheelRPMTargetOut.set(BottomRPMTarget);
    BottomWheelAtTargetRPMOut.set(isBottomWheelAtTargetVelocity());
    BottomWheelCurrentOut.set(BottomShooterMotor.getSupplyCurrent().getValueAsDouble());

  }

  private void motorSetup() {
    //Top Moter Setup
    TalonFXConfiguration topMotor_cfg = new TalonFXConfiguration();
    topMotor_cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    topMotor_cfg.Slot0.kP = Constants.SHOOTER_kP;
    topMotor_cfg.Slot0.kI = Constants.SHOOTER_kI;
    topMotor_cfg.Slot0.kD = Constants.SHOOTER_kD;
    topMotor_cfg.Slot0.kV = Constants.SHOOTER_kV;
    topMotor_cfg.MotionMagic.MotionMagicAcceleration = Constants.SHOOTER_MMAcceleration;
    topMotor_cfg.CurrentLimits.SupplyCurrentLimit = Constants.SHOOTER_CURRENT_LIMIT;
    topMotor_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;


    TopShooterMotor.getConfigurator().apply(topMotor_cfg);

    //Bottom Moter Setup
    TalonFXConfiguration bottomMotor_cfg = new TalonFXConfiguration();
    bottomMotor_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    bottomMotor_cfg.Slot0.kP = Constants.SHOOTER_kP;
    bottomMotor_cfg.Slot0.kI = Constants.SHOOTER_kI;
    bottomMotor_cfg.Slot0.kD = Constants.SHOOTER_kD;
    bottomMotor_cfg.Slot0.kV = Constants.SHOOTER_kV;
    bottomMotor_cfg.MotionMagic.MotionMagicAcceleration = Constants.SHOOTER_MMAcceleration;
    bottomMotor_cfg.CurrentLimits.SupplyCurrentLimit = Constants.SHOOTER_CURRENT_LIMIT;
    bottomMotor_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;


    BottomShooterMotor.getConfigurator().apply(bottomMotor_cfg);

    //topMM.withEnableFOC(false);
    //bottomMM.withEnableFOC(false);
    
    //BottomShooterMotor.setControl(new Follower(TopShooterMotor.getDeviceID(), true));
  }



  public void setShooterSpeed(double speed) {
    motorVoltageRequest.withOutput(12.0 * speed);
    TopShooterMotor.setControl(motorVoltageRequest.withOutput(12.0 * speed * 1.15)); //Added 15% increase to top and 10% to bottom of shooter
    BottomShooterMotor.setControl(motorVoltageRequest.withOutput(12.0 * speed * 1.1));
  }

  public void setShooterRPM(double rpm) {
    setShooterRPM(rpm, rpm);
  }

  public void setShooterRPM(double topRPM, double bottomRPM) {
    TopRPMTarget = topRPM;
    BottomRPMTarget = bottomRPM;

    double topRPS = topRPM / 60.0;
    double bottomRPS = bottomRPM / 60.0;
    
    //TopShooterMotor.setControl(new MotionMagicVelocityVoltage(topRPS, Constants.SHOOTER_kA, false, 0.0, 0, false, false, false));
    //TopShooterMotor.setControl(topMM.withVelocity(topRPS));
    //BottomShooterMotor.setControl(bottomMM.withVelocity(bottomRPS));
    TopShooterMotor.setControl(new MotionMagicVelocityVoltage(topRPS));
    BottomShooterMotor.setControl(new MotionMagicVelocityVoltage(bottomRPS));
  }

  public double getTopMotorRPM() {
    return TopShooterMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getBottomMotorRPM() {
    return BottomShooterMotor.getVelocity().getValueAsDouble() * 60.0;
  }


  public boolean isBottomWheelAtTargetVelocity() {
    return isBottomWheelAtTargetVelocity(Constants.SHOOTER_RPM_TOLERANCE_PERCENT);
  }

  public boolean isBottomWheelAtTargetVelocity(double tolerancePercentage) {
    double actualPercent = Math.abs(getBottomMotorRPM() - BottomRPMTarget) / (BottomRPMTarget + .001);
    return (actualPercent < tolerancePercentage) || getBottomMotorRPM() == Double.NaN;
  }

  public boolean isTopWheelAtTargetVelocity() {
    return isTopWheelAtTargetVelocity(Constants.SHOOTER_RPM_TOLERANCE_PERCENT);
  }

  public boolean isTopWheelAtTargetVelocity(double tolerancePercentage) {
    double actualPercent = Math.abs(getTopMotorRPM() - TopRPMTarget) / (TopRPMTarget + .001);
    return (actualPercent < tolerancePercentage) || getTopMotorRPM() == Double.NaN;
  }

  public double getTopWheelTargetRPM() {
    return TopRPMTarget;
  }
}
