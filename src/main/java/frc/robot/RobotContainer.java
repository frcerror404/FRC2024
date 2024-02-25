// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SetIndexerSpeed;
import frc.robot.commands.SetIntakePosition;
import frc.robot.commands.SetIntakeSpeed;
import frc.robot.commands.SetConveyorSpeed;
import frc.robot.commands.SetShooterAngle;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.enums.IntakePosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;



public class RobotContainer {
  private static final double MaxSpeed = 6; // 6 meters per second desired top speed
  private static final double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity
  private final SendableChooser<Command> autoChooser;


  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driver = new CommandXboxController(0); // My joystick
  private final CommandXboxController operator = new CommandXboxController(1); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Shooter shooter = new Shooter();
  private final Indexer indexer = new Indexer();
  private final ShooterAngle shooterAngle = new ShooterAngle();
  private final IntakePivot intakePivot = new IntakePivot();
  private final Conveyor conveyor = new Conveyor();
  private final Intake intake = new Intake();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driver.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);



    driver.rightBumper()
        .whileTrue(new SetShooterSpeed(shooter, 0.75))
        .whileFalse(new SetShooterSpeed(shooter, 0));

    driver.leftBumper()
        .whileTrue(new SetShooterSpeed(shooter, 1.0))
        .whileFalse(new SetShooterSpeed(shooter, 0));

    driver.a()
        .whileTrue(new SetIndexerSpeed(indexer, 1.0))
        .whileFalse(new SetIndexerSpeed(indexer, 0.0));

    driver.y()
        .whileTrue(new SetShooterAngle(shooterAngle,  Constants.LOCATION_TRUSS))
        .whileFalse(new SetShooterAngle(shooterAngle, Constants.LOCATION_HOME));
    driver.b()
        .whileTrue(new SetShooterAngle(shooterAngle, Constants.LOCATION_TEST))
        .whileFalse(new SetShooterAngle(shooterAngle, Constants.LOCATION_HOME));

    operator.rightBumper()
        .whileTrue(new SetIntakePosition(intakePivot, IntakePosition.DOWN))
        .whileFalse(new SetIntakePosition(intakePivot, IntakePosition.UP));

    operator.a()
        .whileTrue(new SetConveyorSpeed(conveyor, 0.4))
        .whileFalse(new SetConveyorSpeed(conveyor, 0));

    operator.y()
        .whileTrue(new SetIntakeSpeed(intake, 0.5))
        .whileFalse(new SetIntakeSpeed(intake, 0.0));
  }

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void zeroPigeon() {
    // drivetrain.getPigeon2().setYaw(0);
  }
}
