// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SetIndexerSpeed;
import frc.robot.commands.SetShooterAngle;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;



public class RobotContainer {
  private static final double MaxSpeed = 6; // 6 meters per second desired top speed
  private static final double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity
  private final SendableChooser<Command> autoChooser;


  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Shooter shooter = new Shooter();
  private final Conveyor conveyor = new Conveyor();
  private final ShooterAngle shooterAngle = new ShooterAngle();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);



    joystick.rightBumper()
        .whileTrue(new SetShooterSpeed(shooter, 0.75))
        .whileFalse(new SetShooterSpeed(shooter, 0));

    joystick.leftBumper()
        .whileTrue(new SetShooterSpeed(shooter, 1.0))
        .whileFalse(new SetShooterSpeed(shooter, 0));

    joystick.a()
        .whileTrue(new SetIndexerSpeed(conveyor, 0.5))
        .whileFalse(new SetIndexerSpeed(conveyor, 0.0));

    joystick.y()
        .whileTrue(new SetShooterAngle(shooterAngle, 0.06 /*Constants.LOCATION_TRUSS*/))
        .whileFalse(new SetShooterAngle(shooterAngle, 0.00 /*Constants.LOCATION_SUBWOOFER*/));
    // joystick.b()
    //     .whileTrue(new SetShooterAngle(shooterAngle, Constants.LOCATION_WING))
    //     .whileFalse(new SetShooterAngle(shooterAngle, Constants.LOCATION_SUBWOOFER));
  }

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser",autoChooser);
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void zeroPidgeon() {
    drivetrain.getPigeon2().setYaw(0);
  }
}
