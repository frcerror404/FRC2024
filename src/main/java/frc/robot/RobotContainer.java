// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
import frc.robot.commands.IndexerAndConveyor;
import frc.robot.commands.IntakeAndConveyor;
import frc.robot.commands.IntakeUntilNoteDetected;
import frc.robot.commands.SetClimberSpeed;
import frc.robot.commands.SetConveyorSpeed;
import frc.robot.commands.SetShooterAngle;
import frc.robot.commands.SetShooterRPM;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.enums.IntakeConveyorSpeed;
import frc.robot.enums.IntakePosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climbers;
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
  private final Climbers climbers = new Climbers();

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

    // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // driver.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);


    //Added a right bumper shot for the far shot.
    driver.rightBumper()
        .whileTrue(new SetShooterSpeed(shooter, 0.55)) 
        .whileFalse(new SetShooterSpeed(shooter, 0));
    
    // Kept the shot the same
    driver.rightTrigger(0.5)
        .whileTrue(new SetShooterRPM(shooter, Constants.SUBWOOFER_TOP_RPM, Constants.SUBWOOFER_BOTTOM_RPM))
        .whileFalse(new SetShooterRPM(shooter, 0, 0));

    driver.leftTrigger(0.5)
        .whileTrue(new SetShooterRPM(shooter, Constants.AMP_TOP_RPM, Constants.AMP_BOTTOM_RPM))
        .whileFalse(new SetShooterRPM(shooter, 0, 0));

    driver.a()
        .whileTrue(new IndexerAndConveyor(conveyor, indexer, 0.5, 1.0))
        .whileFalse(new IndexerAndConveyor(conveyor, indexer, 0.0, 0.0));

    driver.y()
        .whileTrue(new SetShooterAngle(shooterAngle,  Constants.LOCATION_TRUSS))
        .whileFalse(new SetShooterAngle(shooterAngle, Constants.LOCATION_HOME));
    driver.b()
        .whileTrue(new SetShooterAngle(shooterAngle, Constants.LOCATION_TEST))
        .whileFalse(new SetShooterAngle(shooterAngle, Constants.LOCATION_HOME));

    operator.rightBumper()
        .whileTrue(new SetIntakePosition(intakePivot, IntakePosition.DOWN))
        .whileFalse(new SetIntakePosition(intakePivot, IntakePosition.UP));

    operator.leftBumper()
        .whileTrue(new IntakeUntilNoteDetected(conveyor, intake, 0.4, .5));

    operator.a()
        .whileTrue(new SetConveyorSpeed(conveyor, 0.4))
        .whileFalse(new SetConveyorSpeed(conveyor, 0));

    operator.y()
        .whileTrue(new SetIntakeSpeed(intake, 0.5))
        .whileFalse(new SetIntakeSpeed(intake, 0.0));

    operator.x()
        .whileTrue(new SetIntakeSpeed(intake, -0.6))
        .whileFalse(new SetIntakeSpeed(intake, 0.0));

    operator.b()
        .whileTrue(new IntakeAndConveyor(intake, conveyor, IntakeConveyorSpeed.ON))
        .whileFalse(new IntakeAndConveyor(intake, conveyor, IntakeConveyorSpeed.OFF));

    operator.rightTrigger(.1)
        .whileTrue(new SetClimberSpeed(climbers, 0.5))
        .whileFalse(new SetClimberSpeed(climbers, 0.0));

    operator.leftTrigger(.1)
        .whileTrue(new SetClimberSpeed(climbers, -0.5))
        .whileFalse(new SetClimberSpeed(climbers, 0.0));
  }

  public RobotContainer() {
    // Register the Named Commands for PathPlannerLib
    registerNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void zeroPigeon() {
    drivetrain.getPigeon2().setYaw(0);
  }

  public void setClimberBrakes(boolean enabled) {
    climbers.setBrakeMode(enabled);
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("ShooterOn", new SetShooterRPM(shooter, Constants.SUBWOOFER_TOP_RPM, Constants.SUBWOOFER_BOTTOM_RPM));
    NamedCommands.registerCommand("ShooterOff", new SetShooterRPM(shooter, 0, 0));
    NamedCommands.registerCommand("ConveyorOn", new SetConveyorSpeed(conveyor, 0.35));
    NamedCommands.registerCommand("ConveyorOff", new SetConveyorSpeed(conveyor, 0.0));
    NamedCommands.registerCommand("IndexerOn", new SetIndexerSpeed(indexer, 1.0));
    NamedCommands.registerCommand("IndexerOff", new SetIndexerSpeed(indexer, 0.0));
    NamedCommands.registerCommand("IntakeDown", new SetIntakePosition(intakePivot, IntakePosition.DOWN));
    NamedCommands.registerCommand("IntakeUp", new SetIntakePosition(intakePivot, IntakePosition.UP));
    NamedCommands.registerCommand("IntakeOn", new SetIntakeSpeed(intake, 1.0));
    NamedCommands.registerCommand("IntakeOff", new SetIntakeSpeed(intake, 0.0));
    NamedCommands.registerCommand("AimSubwoofer", new SetShooterAngle(shooterAngle, Constants.LOCATION_SUBWOOFER));
  }
}
