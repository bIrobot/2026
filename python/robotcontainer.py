#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

import commands2
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
import wpimath
import wpilib
from wpilib import DriverStation, SmartDashboard

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
        self.wrist_config.closedLoop.P(10)
        self.wrist_config.absoluteEncoder.inverted(True)
        self.wrist_config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        self.wristMotor.configure(self.wrist_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        self.wristEncoder = self.wristMotor.getAbsoluteEncoder()
        self.wristPidController = self.wristMotor.getClosedLoopController()

        self.leftArmMotor = SparkMax(11, SparkMax.MotorType.kBrushless)
        self.rightArmMotor = SparkMax(12, SparkMax.MotorType.kBrushless)

        self.leftArmConfig = SparkMaxConfig()
        self.leftArmConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        self.leftArmConfig.closedLoop.outputRange(-1, 1)
        self.leftArmConfig.closedLoop.P(10).I(0).D(0)
        self.leftArmConfig.inverted(True)
        self.leftArmMotor.configure(self.leftArmConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        self.rightArmConfig = SparkMaxConfig()
        self.rightArmConfig.inverted(True)
        self.rightArmConfig.follow(11, True)
        self.rightArmMotor.configure(self.rightArmConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        self.armEncoder = self.leftArmMotor.getAbsoluteEncoder()
        self.armPidController = self.leftArmMotor.getClosedLoopController()

        # get the limelight table server
        self.limelightTable = ntcore.NetworkTableInstance.getDefault().getTable("limelight")

        # set the limelight led mode
        self.limelightTable.getEntry("ledMode").setDouble(3)  # on

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

        # Build an auto chooser. This will use Commands.none() as the default option.
        self.autoChooser = AutoBuilder.buildAutoChooser()

        # Another option that allows you to specify the default auto by its name
        # self.autoChooser = AutoBuilder.buildAutoChooser("My Default Auto")

        SmartDashboard.putData("Auto Chooser", self.autoChooser)



    def robotContainerTestPeriodic(self):
        if DriverStation.isEnabled():
            # stop the joystick from driving us (removeDefaultCommand does not work)
            self.robotDrive.setDefaultCommand(commands2.RunCommand(lambda: None, self.robotDrive))
            self.limelightTable.getEntry("ledMode").setDouble(3)  # on

        # periodic code here always runs!
        self.ticks = self.ticks+1
        if self.ticks%50 == 0:
            pass

        if self.driverController.getAButtonPressed():
            print("A has been pressed")
            print("Gyro angle is ", self.robotDrive.getHeading())
            print("Wrist encoder is", self.wristEncoder.getPosition())
            print("arm encoder is", self.armEncoder.getPosition())
            self.armPidController.setReference(0.15, SparkMax.ControlType.kPosition)
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
            self.wristPidController.setReference(0.6, SparkMax.ControlType.kPosition)
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
        elif ta < 1:
            #drive forward faster
            self.robotDrive.drive(0.2, 0, 0, False, True)
            self.fingerMotor.setVoltage(-2.0)

        elif ta < 5:
            # drive forward
            self.robotDrive.drive(0.1, 0, 0, False, True)
            self.fingerMotor.setVoltage(-2.0)
        # otherwise, we have arrived!

        elif ta > 7:
            #drive backwards
            self.robotDrive.drive(-0.1, 0, 0, False, True)
            self.fingerMotor.setVoltage(-2.0)

        else:
            # stop
            self.robotDrive.drive(0, 0, 0, False, True)
            self.fingerMotor.setVoltage(0)

        # remember where we last saw the april tag
        self.lasttx = tx



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
        return self.autoChooser.getSelected()