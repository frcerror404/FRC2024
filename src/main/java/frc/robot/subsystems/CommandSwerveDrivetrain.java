package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

import edu.wpi.first.wpilibj2.command.Subsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private DoublePublisher FLDT, FLST, FRDT, FRST, BLDT, BLST, BRDT, BRST;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configurePathPlanner();

        setUpMotors();

        var result = getPigeon2().setYaw(0.0);
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePathPlanner();

        setUpMotors();
        

        var result = getPigeon2().setYaw(0.0);
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
                new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                        new PIDConstants(2, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> false,
                this); // Subsystem for requirements
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });
    }


    private void setUpMotors() {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        NetworkTable tempTable = nt.getTable("temperature");

        FLDT = tempTable.getDoubleTopic("FL Drive").publish();
        FLST = tempTable.getDoubleTopic("FL Steer").publish();
        FRDT = tempTable.getDoubleTopic("FR Drive").publish();
        FRST = tempTable.getDoubleTopic("FR Steer").publish();
        BLDT = tempTable.getDoubleTopic("BL Drive").publish();
        BLST = tempTable.getDoubleTopic("BL Steer").publish();
        BRDT = tempTable.getDoubleTopic("BR Drive").publish();
        BRST = tempTable.getDoubleTopic("BR Steer").publish();

        CurrentLimitsConfigs driveLimit = new CurrentLimitsConfigs();
        driveLimit.withSupplyCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
        driveLimit.withSupplyCurrentLimitEnable(true);

        CurrentLimitsConfigs steerLimit = new CurrentLimitsConfigs();
        steerLimit.withSupplyCurrentLimit(Constants.STEER_CURRENT_LIMIT);
        steerLimit.withSupplyCurrentLimitEnable(true);

        for(SwerveModule module : Modules) {
            module.getDriveMotor().getConfigurator().apply(driveLimit);
            module.getSteerMotor().getConfigurator().apply(steerLimit);
        }


    }


    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);

    }

    public Command zeroGyro() {
        return run(() -> this.zeroGyro());
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
        } else {
            Logger.recordOutput("SwerveStates/Setpoints", getState().ModuleTargets);
            Logger.recordOutput("SwerveStates/Measured", getState().ModuleStates);
        }

        publishMotorTemps();
    }

    public void publishMotorTemps() {
        FLDT.set(getModule(0).getDriveMotor().getDeviceTemp().getValueAsDouble());
        FLST.set(getModule(0).getSteerMotor().getDeviceTemp().getValueAsDouble());
        FRDT.set(getModule(1).getDriveMotor().getDeviceTemp().getValueAsDouble());
        FRST.set(getModule(1).getSteerMotor().getDeviceTemp().getValueAsDouble());
        BLDT.set(getModule(2).getDriveMotor().getDeviceTemp().getValueAsDouble());
        BLST.set(getModule(2).getSteerMotor().getDeviceTemp().getValueAsDouble());
        BRDT.set(getModule(3).getDriveMotor().getDeviceTemp().getValueAsDouble());
        BRST.set(getModule(3).getSteerMotor().getDeviceTemp().getValueAsDouble());
    }
}
