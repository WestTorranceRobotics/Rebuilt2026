// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.GlobalConstants.*;
import static frc.robot.constants.ShooterConstants.YAW_ACCEPTABLE_ERROR;
import static frc.robot.utilities.CustomUnits.RotationsPerMinute;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Shooter.ShootCommand;
import frc.robot.commands.SwerveDrive.AutonomousPeriodicCommand;
import frc.robot.commands.SwerveDrive.DefaultJoystickCommand;
import frc.robot.constants.SwerveDriveConstants.RealRobotConstants;
import frc.robot.constants.SwerveDriveConstants.RealRobotConstants.RealModuleConstants;
import frc.robot.constants.SwerveDriveConstants.SimulatedControlSystemConstants.SimulatedModuleConstants;
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
import frc.robot.utilities.controller.LogitechController;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation3D;
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
public class RobotContainer {
    private final SwerveDrive swerveDrive;
    private final ShooterIO shooterSubsystem;
    private final IntakeIO intakeSubsystem;
    private final HopperIO hopperSubsystem;

    private final Controller controller;
    private final Controller overrideController;

    public static SwerveDriveSimulation3D swerveDriveSimulation;

    public static VisionIO visionIO;

    private final SendableChooser<Double> shooterSpeedChooser = new SendableChooser<>();
    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * Registers all important robot code, e.g. swerve, path planner, controls
     */
    public RobotContainer() {
        SwerveDriveConfigurator swerveDriveConfigurator;

        for (double startingRPM = 2500; startingRPM < 4600; startingRPM += 50) {
            shooterSpeedChooser.addOption(String.format("%s RPM", startingRPM), startingRPM);
        }
        shooterSpeedChooser.setDefaultOption("Default (3000 RPM)", 3000.0);
        SmartDashboard.putData("Run Shooter at X RPM:", shooterSpeedChooser);

        if (Robot.isReal()) {
            // Real drive train
            var robotConstants = new SwerveDriveConfigurator.SwerveDriveRobotConstants(
                    Kilograms.of(35),
                    Inches.of(30),
                    Inches.of(24.5),
                    Inches.of(2.5),
                    Inches.of(2),
                    RealRobotConstants.PIGEON2_ID);

            swerveDriveConfigurator = new SwerveDriveConfigurator(
                    robotConstants, new SwerveDriveConfigurator.SwerveDriveModuleConstants[] {
                        RealModuleConstants.FLModuleConstants,
                        RealModuleConstants.FRModuleConstants,
                        RealModuleConstants.BLModuleConstants,
                        RealModuleConstants.BRModuleConstants
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

            shooterSubsystem = new ShooterIOReal();
            intakeSubsystem = new IntakeIOReal();
            hopperSubsystem = new HopperIOReal();
            visionIO = new VisionIOReal(swerveDrive::addVisionMeasurement);
        } else {
            // TODO add constant for drive base length
            swerveDriveSimulation = new SwerveDriveSimulation3D(
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

            SwerveDriveConfigurator.SwerveDriveRobotConstants robotConstants =
                    new SwerveDriveConfigurator.SwerveDriveRobotConstants(
                            Pounds.of(75), Inches.of(30), Inches.of(24.5), Inches.of(2.5), Inches.of(2), 0);

            swerveDriveConfigurator = new SwerveDriveConfigurator(
                    robotConstants, new SwerveDriveConfigurator.SwerveDriveModuleConstants[] {
                        SimulatedModuleConstants.FLModuleConstants,
                        SimulatedModuleConstants.FRModuleConstants,
                        SimulatedModuleConstants.BLModuleConstants,
                        SimulatedModuleConstants.BRModuleConstants
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

            swerveDriveSimulation.registerWithArena(Robot.arena, new Pose2d(2, 7, Rotation2d.kZero));
            shooterSubsystem = new ShooterIOSim();
            intakeSubsystem = new IntakeIOSim();
            hopperSubsystem = new HopperIOSim();
            visionIO = new VisionIOSim(swerveDrive::addVisionMeasurement);
        }

        controller = new DualShock4Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);
        overrideController = new LogitechController(OperatorConstants.OVERRIDE_CONTROLLER_PORT);

        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                (stream) -> stream.filter(auto -> !auto.getName().startsWith("test")));
        SmartDashboard.putData("Current Auto:", autoChooser);

        configureBindings();
    }

    private void registerNamedCommands() {
        var alignCommand = Commands.startRun(
                        () -> {
                            swerveDrive.setAlignStatus(true, 0);
                            // PPHolonomicDriveController.overrideRotationFeedback(() ->
                            // -swerveDrive.getChassisSpeed().omegaRadiansPerSecond);
                        },
                        () -> {
                            swerveDrive.drive(new ChassisSpeeds(), true);
                        })
                .until(() -> {
                    var apriltagId = isAllianceBlue() ? 25 : 10;
                    var yaw = visionIO.getTX(apriltagId);
                    if (yaw.isEmpty()) {
                        SmartDashboard.putNumber("Yaw from target", -1);
                        return true; // Break because no AprilTag was found
                    }
                    SmartDashboard.putNumber("Yaw from target", yaw.get());
                    swerveDrive.setAlignStatus(true, yaw.get());
                    return Math.abs(yaw.get())
                            < YAW_ACCEPTABLE_ERROR; // Break when we're within acceptable yaw to shoot
                })
                .finallyDo(() -> {
                    swerveDrive.setAlignStatus(false, 0);
                    swerveDrive.drive(new ChassisSpeeds(), true);
                });
        NamedCommands.registerCommand("align", alignCommand);

        NamedCommands.registerCommand(
                "alignAndShoot",
                Commands.parallel(
                        Commands.run(() -> {
                            swerveDrive.drive(new ChassisSpeeds(), true);
                        }),
                        new ShootCommand(shooterSubsystem, swerveDrive, visionIO, hopperSubsystem).withTimeout(6)));

        NamedCommands.registerCommand("startIntake", intakeSubsystem.runOnce(intakeSubsystem::intake));

        NamedCommands.registerCommand("stopIntake", intakeSubsystem.runOnce(intakeSubsystem::stopIntake));

        NamedCommands.registerCommand(
                "pivotDown", intakeSubsystem.sendHoodDownCommand().withTimeout(0.75));

        NamedCommands.registerCommand(
                "pivotUp", intakeSubsystem.sendHoodUpCommand().withTimeout(0.75));
    }

    private void configureBindings() {
        // reset the heading of the swerve
        controller.zero().onTrue(Commands.runOnce(this::zeroHeading));

        // shooter button mapping
        controller.aOrCross().whileTrue(new ShootCommand(shooterSubsystem, swerveDrive, visionIO, hopperSubsystem));

        // align button mapping
        controller
                .bOrCircle()
                .whileTrue(Commands.run(() -> {
                    if (visionIO.getBestTarget() == null) return;
                    // align robot to best AprilTag
                    //     swerveDrive.setAlignStatus(
                    //             true,
                    //             visionIO.getTX(visionIO.getBestTarget().getFiducialId())
                    //                     .orElse(null));
                    int hubAprilTagID = isAllianceBlue() ? 25 : 10;
                    if (visionIO.getTX(hubAprilTagID).orElse(null) != null) {
                        swerveDrive.setAlignStatus(
                                true, visionIO.getTX(hubAprilTagID).get());
                    }
                }))
                .onFalse(Commands.run(() -> {
                    swerveDrive.setAlignStatus(false, 0);
                }));

        controller.xOrSquare().toggleOnTrue(intakeSubsystem.intakeCommand());
        controller.yOrTriangle().toggleOnTrue(intakeSubsystem.outtakeCommand());

        controller.dPadUp().whileTrue(intakeSubsystem.sendHoodUpCommand());
        controller.dPadDown().whileTrue(intakeSubsystem.sendHoodDownCommand());

        overrideController
                .aOrCross()
                .whileTrue(Commands.parallel(
                        hopperSubsystem.runHopperCommand(),
                        shooterSubsystem.runShooterCommand(RotationsPerMinute.of(2700.0))));

        overrideController
                .bOrCircle()
                .whileTrue(Commands.parallel(
                        hopperSubsystem.runHopperCommand(),
                        shooterSubsystem.runShooterCommand(RotationsPerMinute.of(3850.0))));

        overrideController
                .xOrSquare()
                .whileTrue(Commands.parallel(
                        hopperSubsystem.runHopperCommand(),
                        shooterSubsystem.runShooterCommand(RotationsPerMinute.of(4316.6667))));

        overrideController
                .yOrTriangle()
                .whileTrue(Commands.parallel(
                        hopperSubsystem.runHopperCommand(),
                        shooterSubsystem.runShooterCommand(RotationsPerMinute.of(4767.0))));
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
        try {
            swerveDrive.getDefaultCommand().cancel();
        } catch (NullPointerException e) {
            DriverStation.reportError("Tried to unbind non existent default command", true);
        }

        swerveDrive.removeDefaultCommand();
    }

    public void bindAutoCommand() {
        swerveDrive.setDefaultCommand(new AutonomousPeriodicCommand(swerveDrive));
    }

    /**
     * Gets path planner auto to be run during autonomous.
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Stops the robot's movement.
     */
    public void clearModuleStates() {
        swerveDrive.drive(new ChassisSpeeds(), true);
    }
}
