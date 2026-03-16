// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.GlobalConstants.OperatorConstants.DRIVER_CONTROLLER_PORT;
import static frc.robot.utilities.CustomUnits.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwerveDrive.DefaultJoystickCommand;
import frc.robot.constants.SwerveDriveConstants;
import frc.robot.constants.SwerveDriveConstants.RealRobotConstants;
import frc.robot.subsystems.Hopper.HopperIO;
import frc.robot.subsystems.Hopper.HopperIOReal;
import frc.robot.subsystems.Hopper.HopperIOSim;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeIOReal;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIOReal;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.SwerveDriveConfigurator;
import frc.robot.subsystems.hardware.gyroscope.GyroIOPigeon2;
import frc.robot.subsystems.hardware.gyroscope.GyroIOSim;
import frc.robot.subsystems.hardware.module.ModuleIOReal;
import frc.robot.subsystems.hardware.module.ModuleIOSim;
import frc.robot.subsystems.hardware.vision.VisionIO;
import frc.robot.subsystems.hardware.vision.VisionIOReal;
import frc.robot.subsystems.hardware.vision.VisionIOSim;
import frc.robot.utilities.controller.Controller;
import frc.robot.utilities.controller.DualShock4Controller;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
    private final SwerveDrive swerveDrive;
    private final ShooterIO shooterSubsystem;
    private final IntakeIO intakeSubsystem;
    private final HopperIO hopperSubsystem;

    private final Controller controller;

    public static SwerveDriveSimulation swerveDriveSimulation;

    public static VisionIO visionIO;

    private final SendableChooser<Double> m_chooser = new SendableChooser<>(); // prepare to build LUT

    public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(-25.0 / 2, -19.5 / 2),
            new Translation2d(-25.0 / 2, 19.5 / 2),
            new Translation2d(25.0 / 2, -19.5 / 2),
            new Translation2d(25.0 / 2, 19.5 / 2));

    /**
     * Registers all important robot code, e.g. swerve, path planner, controls
     */
    public RobotContainer() {
        SwerveDriveConfigurator swerveDriveConfigurator;

        for (double startingRPM = 2500; startingRPM < 4600; startingRPM += 50) {
            m_chooser.addOption(String.format("%s RPM", startingRPM), startingRPM);
        }
        m_chooser.setDefaultOption("Default (3000 RPM)", 3000.0);
        SmartDashboard.putData("Run Shooter at X RPM:", m_chooser);

        if (Robot.isReal()) {
            // Real drive train
            SwerveDriveConfigurator.SwerveDriveModuleConstants FLModuleConstants =
                    new SwerveDriveConfigurator.SwerveDriveModuleConstants(
                            SwerveDriveConfigurator.SwerveModuleCornerPosition.FRONT_LEFT,
                            RealRobotConstants.FL_CANCODER_ID,
                            RealRobotConstants.FL_DRIVE_MOTOR_ID,
                            RealRobotConstants.FL_AZIMUTH_MOTOR_ID,
                            RealRobotConstants.FL_CANCODER_OFFSET,
                            RealRobotConstants.kPDrive,
                            RealRobotConstants.kIDrive,
                            RealRobotConstants.kDDrive,
                            RealRobotConstants.kSDrive,
                            RealRobotConstants.kVDrive,
                            RealRobotConstants.kPAzimuth,
                            RealRobotConstants.kIAzimuth,
                            RealRobotConstants.kDAzimuth,
                            0,
                            3,
                            false,
                            1 / 6.2);
            SwerveDriveConfigurator.SwerveDriveModuleConstants FRModuleConstants =
                    new SwerveDriveConfigurator.SwerveDriveModuleConstants(
                            FLModuleConstants,
                            SwerveDriveConfigurator.SwerveModuleCornerPosition.FRONT_RIGHT,
                            RealRobotConstants.FR_CANCODER_ID,
                            RealRobotConstants.FR_DRIVE_MOTOR_ID,
                            RealRobotConstants.FR_AZIMUTH_MOTOR_ID,
                            RealRobotConstants.FR_CANCODER_OFFSET);
            SwerveDriveConfigurator.SwerveDriveModuleConstants BLModuleConstants =
                    new SwerveDriveConfigurator.SwerveDriveModuleConstants(
                            FLModuleConstants,
                            SwerveDriveConfigurator.SwerveModuleCornerPosition.BACK_LEFT,
                            RealRobotConstants.BL_CANCODER_ID,
                            RealRobotConstants.BL_DRIVE_MOTOR_ID,
                            RealRobotConstants.BL_AZIMUTH_MOTOR_ID,
                            RealRobotConstants.BL_CANCODER_OFFSET);
            SwerveDriveConfigurator.SwerveDriveModuleConstants BRModuleConstants =
                    new SwerveDriveConfigurator.SwerveDriveModuleConstants(
                            FLModuleConstants,
                            SwerveDriveConfigurator.SwerveModuleCornerPosition.BACK_RIGHT,
                            RealRobotConstants.BR_CANCODER_ID,
                            RealRobotConstants.BR_DRIVE_MOTOR_ID,
                            RealRobotConstants.BR_AZIMUTH_MOTOR_ID,
                            RealRobotConstants.BR_CANCODER_OFFSET);

            SwerveDriveConfigurator.SwerveDriveRobotConstants robotConstants =
                    new SwerveDriveConfigurator.SwerveDriveRobotConstants(
                            Kilograms.of(35),
                            Inches.of(30),
                            Inches.of(24.5),
                            Inches.of(2.5),
                            Inches.of(2),
                            RealRobotConstants.PIGEON2_ID);

            swerveDriveConfigurator = new SwerveDriveConfigurator(
                    robotConstants, new SwerveDriveConfigurator.SwerveDriveModuleConstants[] {
                        FLModuleConstants, FRModuleConstants, BLModuleConstants, BRModuleConstants
                    });

            swerveDrive = new SwerveDrive(
                    new GyroIOPigeon2(robotConstants.pigeonID),
                    new ModuleIOReal(
                            SwerveDriveConfigurator.SwerveModuleCornerPosition.FRONT_LEFT, swerveDriveConfigurator),
                    new ModuleIOReal(
                            SwerveDriveConfigurator.SwerveModuleCornerPosition.FRONT_RIGHT, swerveDriveConfigurator),
                    new ModuleIOReal(
                            SwerveDriveConfigurator.SwerveModuleCornerPosition.BACK_LEFT, swerveDriveConfigurator),
                    new ModuleIOReal(
                            SwerveDriveConfigurator.SwerveModuleCornerPosition.BACK_RIGHT, swerveDriveConfigurator));

            controller = new DualShock4Controller(DRIVER_CONTROLLER_PORT);
            shooterSubsystem = new ShooterIOReal();
            intakeSubsystem = new IntakeIOReal();
            hopperSubsystem = new HopperIOReal();
            visionIO = new VisionIOReal();
        } else {

            // TODO add constant for drive base length
            swerveDriveSimulation = new SwerveDriveSimulation(
                    DriveTrainSimulationConfig.Default()
                            .withRobotMass(Pounds.of(75))
                            .withSwerveModule(COTS.ofSwerveX2(
                                    DCMotor.getKrakenX60(1),
                                    DCMotor.getNEO(1),
                                    COTS.WHEELS.SLS_PRINTED_WHEELS.cof,
                                    2,
                                    11))
                            .withTrackLengthTrackWidth(Inches.of(30 - 5), Inches.of(24.5 - 5))
                            .withBumperSize(Inches.of(31), Inches.of(31)),
                    new Pose2d(2, 7, Rotation2d.kZero));

            SwerveDriveConfigurator.SwerveDriveModuleConstants FLModuleConstants =
                    new SwerveDriveConfigurator.SwerveDriveModuleConstants(
                            SwerveDriveConfigurator.SwerveModuleCornerPosition.FRONT_LEFT,
                            0,
                            0,
                            0,
                            0,
                            SwerveDriveConstants.SimulatedControlSystemConstants.kPDrive,
                            SwerveDriveConstants.SimulatedControlSystemConstants.kIDrive,
                            SwerveDriveConstants.SimulatedControlSystemConstants.kDDrive,
                            0,
                            SwerveDriveConstants.SimulatedControlSystemConstants.kVDrive,
                            SwerveDriveConstants.SimulatedControlSystemConstants.kPSteer,
                            SwerveDriveConstants.SimulatedControlSystemConstants.kISteer,
                            SwerveDriveConstants.SimulatedControlSystemConstants.kDSteer,
                            0,
                            2,
                            false,
                            1 / 6.2);
            SwerveDriveConfigurator.SwerveDriveModuleConstants FRModuleConstants =
                    new SwerveDriveConfigurator.SwerveDriveModuleConstants(
                            FLModuleConstants,
                            SwerveDriveConfigurator.SwerveModuleCornerPosition.FRONT_RIGHT,
                            0,
                            0,
                            0,
                            0);
            SwerveDriveConfigurator.SwerveDriveModuleConstants BLModuleConstants =
                    new SwerveDriveConfigurator.SwerveDriveModuleConstants(
                            FLModuleConstants,
                            SwerveDriveConfigurator.SwerveModuleCornerPosition.BACK_LEFT,
                            0,
                            0,
                            0,
                            0);
            SwerveDriveConfigurator.SwerveDriveModuleConstants BRModuleConstants =
                    new SwerveDriveConfigurator.SwerveDriveModuleConstants(
                            FLModuleConstants,
                            SwerveDriveConfigurator.SwerveModuleCornerPosition.BACK_RIGHT,
                            0,
                            0,
                            0,
                            0);

            SwerveDriveConfigurator.SwerveDriveRobotConstants robotConstants =
                    new SwerveDriveConfigurator.SwerveDriveRobotConstants(
                            Pounds.of(75), Inches.of(30), Inches.of(24.5), Inches.of(2.5), Inches.of(2), 0);

            swerveDriveConfigurator = new SwerveDriveConfigurator(
                    robotConstants, new SwerveDriveConfigurator.SwerveDriveModuleConstants[] {
                        FLModuleConstants, FRModuleConstants, BLModuleConstants, BRModuleConstants
                    });

            // TODO change this to not assume square drivebase
            swerveDrive = new SwerveDrive(
                    new GyroIOSim(swerveDriveSimulation.getGyroSimulation()),
                    new ModuleIOSim(
                            swerveDriveSimulation.getModules()[0],
                            SwerveDriveConfigurator.SwerveModuleCornerPosition.FRONT_LEFT,
                            swerveDriveConfigurator),
                    new ModuleIOSim(
                            swerveDriveSimulation.getModules()[1],
                            SwerveDriveConfigurator.SwerveModuleCornerPosition.FRONT_RIGHT,
                            swerveDriveConfigurator),
                    new ModuleIOSim(
                            swerveDriveSimulation.getModules()[2],
                            SwerveDriveConfigurator.SwerveModuleCornerPosition.BACK_LEFT,
                            swerveDriveConfigurator),
                    new ModuleIOSim(
                            swerveDriveSimulation.getModules()[3],
                            SwerveDriveConfigurator.SwerveModuleCornerPosition.BACK_RIGHT,
                            swerveDriveConfigurator));

            SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
            controller = new DualShock4Controller(DRIVER_CONTROLLER_PORT);
            shooterSubsystem = new ShooterIOSim();
            intakeSubsystem = new IntakeIOSim();
            hopperSubsystem = new HopperIOSim();
            visionIO = new VisionIOSim();
        }

        NamedCommands.registerCommand(
                "alignToHub",
                new InstantCommand(() -> swerveDrive.turnToYaw(
                        visionIO.getTX(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue) ? 25 : 10)
                                .orElse(null))));
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
     * for
     * {@link CommandXboxController Xbox}/{@link
     * edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers
     * or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // reset the heading of the swerve
        controller.zero().onTrue(Commands.runOnce(this::zeroHeading));

        // shooter button mapping

        controller
                .a()
                .onTrue(shooterSubsystem.runOnce(() -> {
                    shooterSubsystem.setFeederVoltageDirectly(Volts.of(.75));
                    int hubAprilTagID = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue) ? 25 : 10;
                    // if (visionIO.getTX(hubAprilTagID).isPresent()) {
                    //                        Translation2d hubPosition = visionIO.getTargetPose(hubAprilTagID)
                    //                                .orElse(null)
                    //                                .getTranslation();
                    //
                    //                        ChassisSpeeds robotVelocityTranslation =
                    // m_swerveDrive.getChassisSpeed();
                    //                        Translation2d robotVelocity = new Translation2d(
                    //                                -robotVelocityTranslation.vxMetersPerSecond,
                    //                                -robotVelocityTranslation.vyMetersPerSecond);
                    //
                    //                        Translation2d futurePosition = swerveDriveSimulation
                    //                                .getSimulatedDriveTrainPose()
                    //                                .getTranslation()
                    //                                .plus(robotVelocity.times(LATENCY_COMPENSATION));
                    //
                    //                        Translation2d distanceFromTarget = hubPosition.minus(futurePosition);
                    //
                    //                        double baseHorizontalVelocity =
                    //                                distanceFromTarget.getNorm() /
                    // DISTANCE_TO_TOF_MAP.get(distanceFromTarget.getNorm());
                    //
                    //                        double targetHorizontalVelocity = distanceFromTarget
                    //                                .div(distanceFromTarget.getNorm())
                    //                                .times(baseHorizontalVelocity)
                    //                                .minus(robotVelocity)
                    //                                .getNorm();
                    /* we realistically want to prioritize changing our hood angle rather than the flywheel speed
                    because our flywheel recovery speed is slow */
                    // shooterSubsystem.setFlywheelSpeed(
                    //         RotationsPerMinute.of(shooterMap.get(targetHorizontalVelocity)));
                    shooterSubsystem.setFlywheelSpeed(RotationsPerMinute.of(m_chooser.getSelected()));
                    // }
                }))
                .onFalse(shooterSubsystem.runOnce(shooterSubsystem::stopFlywheel));

        controller.b().whileTrue(shooterSubsystem.run(() -> {
            // swerveDrive.turnToYaw(visionIO.getTX(visionIO.getBestTarget().getFiducialId()).orElse(null));
            // align robot to best AprilTag

            int hubAprilTagID = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue) ? 25 : 10;
            if (visionIO.getTX(hubAprilTagID).isPresent()) {
                swerveDrive.turnToYaw(visionIO.getTX(hubAprilTagID).get());
            }
        }));

        // intake button mapping
        controller.x().onTrue(intakeSubsystem.runOnce(() -> {
            if (intakeSubsystem.isIntakeOn()) {
                intakeSubsystem.stopIntake();
            } else {
                intakeSubsystem.setIntakeVoltage(Volts.of(.75));
            }
        }));

        controller.y().onTrue(intakeSubsystem.runOnce(() -> {
            intakeSubsystem.setHoodVoltage(Volts.of(3));
        })).onFalse(intakeSubsystem.run(intakeSubsystem::stopHoodCommand));

        hopperSubsystem.setRollerVoltage(Volts.of(.75));
    }

    /**
     * Sets the controller as the default movement command for swerve.
     */
    public void bindJoystickCommand() {
        swerveDrive.setDefaultCommand(new DefaultJoystickCommand(
                controller::getLeftX, controller::getLeftY, controller::getRightX, swerveDrive));
        // m_swerveDrive.setDefaultCommand(new SysIDCommand(m_swerveDrive,
        // SysIDCommand.Routine.DRIVE_VELOCITY_DYNAMIC, controller));
    }

    public void zeroHeading() {
        swerveDrive.zeroHeading();
    }

    /**
     * Removes the controller from being used for movement.
     */
    public void unbindJoystick() {
        swerveDrive.removeDefaultCommand();
    }

    /**
     * Gets path planner auto to be run during autonomous.
     */
    public PathPlannerAuto getAutonomousCommand() {
        return new PathPlannerAuto("New New Auto");
    }

    /**
     * Stops the robot's movement.
     */
    public void clearModuleStates() {
        swerveDrive.drive(new ChassisSpeeds(), true);
    }
}
