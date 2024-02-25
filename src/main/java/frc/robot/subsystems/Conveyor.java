// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
  CANSparkMax TopConveyorMotor = new CANSparkMax(Constants.CONVEYOR_TOP_ID, MotorType.kBrushless);
  CANSparkMax BottomConveyorMotor = new CANSparkMax(Constants.CONVEYOR_BOTTOM_ID, MotorType.kBrushless);


  double top_max_speed_ratio = (Constants.NEO550_MAX_SPEED / Constants.CONVEYOR_BOTTOM_GEAR_RATIO) / (Constants.NEO_MAX_SPEED / Constants.CONVEYOR_TOP_GEAR_RATIO);
  double top_cir = Math.PI * Constants.CONVEYOR_TOP_WHEEL_DIA;
  double bottom_cir = Math.PI * Constants.CONVEYOR_BOTTOM_WHEEL_DIA;
  // SFM = ( RPM * Diameter ) * (PI / 12)
  double top_max_sfm = (Constants.NEO_MAX_SPEED / Constants.CONVEYOR_TOP_GEAR_RATIO) * Constants.CONVEYOR_TOP_WHEEL_DIA * (Math.PI / 12.0);
  double bottom_max_sfm = (Constants.NEO550_MAX_SPEED / Constants.CONVEYOR_BOTTOM_GEAR_RATIO) * Constants.CONVEYOR_BOTTOM_WHEEL_DIA * (Math.PI / 12.0);
  double top_sfm_ratio = bottom_max_sfm / top_max_sfm;
  
  
  
  /** Creates a new Conveyor. */
  public Conveyor() {
    BottomConveyorMotor.setInverted(true);
    TopConveyorMotor.setInverted(true);
  }

  public void SetConveyorSpeed(double speed) {
    TopConveyorMotor.set(speed * top_sfm_ratio);
    BottomConveyorMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
