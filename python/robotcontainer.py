#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

import commands2
import wpimath
import wpilib
from wpilib import DriverStation

import ntcore

from rev import ClosedLoopConfig, SparkBase, SparkBaseConfig, SparkMax, SparkMaxConfig

from commands2 import cmd
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import (
    TrajectoryConfig,
    TrajectoryGenerator,
    TrapezoidProfileRadians,
)
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)

from constants import AutoConstants, DriveConstants, ModuleConstants, OIConstants
from subsystems.drivesubsystem import DriveSubsystem


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()

        # The driver's controller
        self.driverController = wpilib.XboxController(OIConstants.kDriverControllerPort)

        # Configure the button bindings
        self.configureButtonBindings()

        self.fingerMotor = SparkMax(14, SparkMax.MotorType.kBrushless)


        self.wristMotor = SparkMax(13, SparkMax.MotorType.kBrushless)
        self.wrist_config = SparkMaxConfig()
        self.wrist_config.setIdleMode(SparkBaseConfig.IdleMode.kCoast)
        self.wrist_config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        self.wristMotor.configure(self.wrist_config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)
        self.wristEncoder = self.wristMotor.getAbsoluteEncoder()
        self.wristPidController = self.wristMotor.getClosedLoopController()


        # get the limelight table server
        self.limelightTable = ntcore.NetworkTableInstance.getDefault().getTable("limelight")

        # set the limelight led mode
        self.limelightTable.getEntry("ledMode").setDouble(2)  # blink

        # count our periodic calls
        self.ticks = 0

        self.lasttx = 0
        self.seeking = False

        # Configure default commands
        self.robotDrive.setDefaultCommand(
            # The left stick controls translation of the robot.
            # Turning is controlled by the X axis of the right stick.
            commands2.RunCommand(
                lambda: self.robotDrive.drive(
                    -wpimath.applyDeadband(
                        self.driverController.getLeftY(), OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getLeftX(), OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getRightX(), OIConstants.kDriveDeadband
                    ),
                    False,
                    True,
                ),
                self.robotDrive,
            )
        )

    def robotContainerTestPeriodic(self):
        if DriverStation.isEnabled():
            # stop the joystick from driving us (removeDefaultCommand does not work)
            self.robotDrive.setDefaultCommand(commands2.RunCommand(lambda: None, self.robotDrive))
            self.limelightTable.getEntry("ledMode").setDouble(3)  # on

        if self.driverController.getAButtonPressed():
            print("A has been pressed")
            print("Gyro angle is ", self.robotDrive.getHeading())
            print("Wrist encoder is", self.wristEncoder.getPosition())
        if self.driverController.getAButtonReleased():
            print("A has been released")

        if self.driverController.getBButtonPressed():
            print("B has been pressed")
            # get the limelight apriltag vision results
            tv = self.limelightTable.getEntry("tv").getDouble(-1000)
            tx = self.limelightTable.getEntry("tx").getDouble(-1000)
            ty = self.limelightTable.getEntry("ty").getDouble(-1000)
            ta = self.limelightTable.getEntry("ta").getDouble(-1000)
            # when tv is 1, we see an apriltag!
            print("tv = ", tv, "tx = ", tx, "; ty = ", ty, "ta = ", ta)
            self.seeking = True
        if self.driverController.getBButtonReleased():
            print("B has been released")

        if self.driverController.getYButtonPressed():
            print("Y has been pressed")
            self.wristPidController.setReference(0.3, SparkMax.ControlType.kPosition)
        if self.driverController.getYButtonReleased():
            print("Y has been released")

        if self.driverController.getXButtonPressed():
            print("X has been pressed")
            self.wristPidController.setReference(0.4, SparkMax.ControlType.kPosition)
        if self.driverController.getXButtonReleased():
            print("X has been released")


        if self.seeking != True:
            return

        tv = self.limelightTable.getEntry("tv").getDouble(-1000)
        tx = self.limelightTable.getEntry("tx").getDouble(-1000)
        ty = self.limelightTable.getEntry("ty").getDouble(-1000)
        ta = self.limelightTable.getEntry("ta").getDouble(-1000)

        # if we don't see the april tag at all...
        if tv!=1:
            if self.lasttx < 0:
                # turn left
                self.robotDrive.drive(0, 0, 0.1, False, True)
                self.fingerMotor.setVoltage(-2.0)
            else:
                # turn right
                self.robotDrive.drive(0, 0, -0.1, False, True)
                self.fingerMotor.setVoltage(-2.0)
            return

        # otherwise, if we see the april tag to our right...
        elif (tx >= 5):
            # turn right
            self.robotDrive.drive(0, 0, -0.1, False, True)
            self.fingerMotor.setVoltage(-2.0)
        # otherwise, if we see the april tag to our left...
        elif (tx <= -5):
            # turn left
            self.robotDrive.drive(0, 0, 0.1, False, True)
            self.fingerMotor.setVoltage(-2.0)
        # otherwise, if we see the april tag dead-ahead but it is too small...
        elif ta < 5:
            # drive forward
            self.robotDrive.drive(0.1, 0, 0, False, True)
            self.fingerMotor.setVoltage(-2.0)
        # otherwise, we have arrived!
        else:
            # stop
            self.robotDrive.drive(0, 0, 0, False, True)
            self.fingerMotor.setVoltage(0)

        # remember where we last saw the april tag
        self.lasttx = tx

        # periodic code here always runs!
        self.ticks = self.ticks+1
        if self.ticks%50 == 0:
            print("tx=", tx, "ta=", ta)

        # fix -- make sure we stop when we approach the april tag at the end of the challenge

        # todo -- when the april tag leaves our view from the left, we have to rotate to the left to find it.
        #         when the april tag leaves our view from the right, we have to rotate to the right to find it.
        #         to do this, we'll need a member variable with the last "tx" value we saw when tv was 1!



    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        # Create config for trajectory
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        )
        # Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics)

        # An example trajectory to follow. All units in meters.
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the origin facing the +X direction
            Pose2d(0, 0, Rotation2d(0)),
            # Pass through these two interior waypoints, making an 's' curve path
            [Translation2d(1, 1), Translation2d(2, -1)],
            # End 3 meters straight ahead of where we started, facing forward
            Pose2d(3, 0, Rotation2d(0)),
            config,
        )

        # Constraint for the motion profiled robot angle controller
        kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
            AutoConstants.kMaxAngularSpeedRadiansPerSecond,
            AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared,
        )

        kPXController = PIDController(1.0, 0.0, 0.0)
        kPYController = PIDController(1.0, 0.0, 0.0)
        kPThetaController = ProfiledPIDControllerRadians(
            1.0, 0.0, 0.0, kThetaControllerConstraints
        )
        kPThetaController.enableContinuousInput(-math.pi, math.pi)

        kPIDController = HolonomicDriveController(
            kPXController, kPYController, kPThetaController
        )

        swerveControllerCommand = commands2.SwerveControllerCommand(
            exampleTrajectory,
            self.robotDrive.getPose,  # Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            # Position controllers
            kPIDController,
            self.robotDrive.setModuleStates,
            (self.robotDrive,),
        )

        # Reset odometry to the starting pose of the trajectory.
        self.robotDrive.resetOdometry(exampleTrajectory.initialPose())

        # Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(
            cmd.run(
                lambda: self.robotDrive.drive(0, 0, 0, False, False),
                self.robotDrive,
            )
        )
