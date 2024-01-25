// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX MotorA = new TalonFX(50);
   TalonFX MotorB = new TalonFX(51);
  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setShooterSpeed(double speed) {
    MotorA.set(speed * .75);
     MotorB.set(-speed);
  } 
}
