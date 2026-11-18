// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.system.RobotController;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import org.wpilib.system.Threads;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Command.InterruptionBehavior;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.command2.button.RobotModeTriggers;

import java.util.Set;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Autoaim;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeRack;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.RollerFloor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelights;

public class Robot extends LoggedRobot {

    private final Drivetrain drivetrain;
    private final Hood hood;
    private final Indexer indexer;
    private final IntakeRack intakeRack;
    private final IntakeRoller intakeRoller;
    private final RollerFloor rollerFloor;
    private final Shooter shooter;

    private final Autoaim autoaim;

    private final VisionIO visionIO;
    private final Vision vision;

    private final SubsystemManager subsystemManager;

    private final RobotContainer robotContainer;

    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    private final Autos autos;

    private CommandXboxController joystick = new CommandXboxController(0);

    public Robot() {

        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher()); 

        SignalLogger.enableAutoLogging(false);

        drivetrain = new Drivetrain(
            joystick,
            TunerConstants.DrivetrainConstants, 
            TunerConstants.FrontLeft, 
            TunerConstants.FrontRight, 
            TunerConstants.BackLeft, 
            TunerConstants.BackRight);

        autoaim = new Autoaim(drivetrain);

        hood = new Hood(autoaim::getHoodPositionToTargetWhileMoving);
        indexer = new Indexer();
        intakeRack = new IntakeRack();
        intakeRoller = new IntakeRoller();
        rollerFloor = new RollerFloor();
        shooter = new Shooter(autoaim::getShooterSpeedToTargetWhileMoving);

        this.visionIO = new VisionIOLimelights();
        this.vision = new Vision(visionIO, drivetrain);

        subsystemManager = new SubsystemManager(autoaim, drivetrain, hood, indexer, intakeRack, intakeRoller, rollerFloor, shooter, joystick);

        robotContainer = new RobotContainer(joystick, autoaim, subsystemManager, drivetrain, hood, indexer, intakeRack, intakeRoller, rollerFloor, shooter);

        this.autoFactory = new AutoFactory(
            drivetrain::getPose, // A function that returns the current robot pose
            drivetrain::resetPose, // A function that resets the current robot pose to the provided Pose2d
            drivetrain::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            drivetrain // The drive subsystem
        );

        autos = new Autos(autoFactory, subsystemManager);
        
        autoChooser = new AutoChooser();

        autoChooser.addRoutine("LT_DoubleSweepReturn", autos::LT_DoubleSweepReturn);
        autoChooser.addRoutine("RT_DoubleSweepReturn", autos::RT_DoubleSweepReturn);
        autoChooser.addRoutine("LT_DoubleSweepReturnSecondScoopCloseToHub", autos::LT_DoubleSweepReturnSecondScoopCloseToHub);
        autoChooser.addRoutine("RT_DoubleSweepReturnSecondScoopCloseToHub", autos::RT_DoubleSweepReturnSecondScoopCloseToHub);
        autoChooser.addRoutine("LT_DoubleSweepReturnSecondScoopShorter", autos::LT_DoubleSweepReturnSecondScoopShorter);
        autoChooser.addRoutine("RT_DoubleSweepReturnSecondScoopShorter", autos::RT_DoubleSweepReturnSecondScoopShorter);
        autoChooser.addRoutine("LT_DoubleSweepReturnSecondScoopSnake", autos::LT_DoubleSweepReturnSecondScoopSnake);
        autoChooser.addRoutine("RT_DoubleSweepReturnSecondScoopSnake", autos::RT_DoubleSweepReturnSecondScoopSnake);
        autoChooser.addRoutine("LT_DoubleSweepReturnSecondScoopLoop", autos::LT_DoubleSweepReturnSecondScoopLoop);
        autoChooser.addRoutine("RT_DoubleSweepReturnSecondScoopLoop", autos::RT_DoubleSweepReturnSecondScoopLoop);
        autoChooser.addRoutine("LToffset_DoubleSweepReturn", autos::LToffset_DoubleSweepReturn);
        autoChooser.addRoutine("RToffset_DoubleSweepReturn", autos::RToffset_DoubleSweepReturn);
        autoChooser.addRoutine("LB_SingleSweep", autos::LB_SingleSweep);
        autoChooser.addRoutine("RB_SingleSweep", autos::RB_SingleSweep);
        autoChooser.addRoutine("M_Preload", autos::M_Preload);
        
        TunableDouble autoStartDelay = Tunables.addDouble("AutoStartDelay", 0.0);

        RobotModeTriggers.autonomous().whileTrue(
            Commands.defer(
                () -> Commands.waitSeconds(
                        Math.max(0.0, autoStartDelay.get()))
                    .andThen(autoChooser.selectedCommandScheduler())
                    .withInterruptBehavior(InterruptionBehavior.CANCEL_INCOMING),
                Set.of()));
        RobotModeTriggers.autonomous().onTrue(Commands.waitSeconds(0.2).andThen(Commands.run(() -> vision.enableVision())));

        Tunables.publish("AutoChooser", autoChooser);

        Threads.setCurrentThreadPriority(5);

        Logger.start();

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run(); 

        vision.periodicRun();
        subsystemManager.periodicRun();
        autoaim.periodicRun();

        drivetrain.periodicRun();
        hood.periodicRun();
        indexer.periodicRun();
        intakeRack.periodicRun();
        intakeRoller.periodicRun();
        rollerFloor.periodicRun();
        shooter.periodicRun();

        var latestShiftInfo = HubShiftUtil.getOfficialShiftInfo();
        Logger.recordOutput("HubShift/Official", latestShiftInfo);
        Logger.recordOutput("BatteryVoltage", RobotController.getBatteryVoltage());

        Logger.recordOutput("Current Shift", latestShiftInfo.currentShift().toString());
        // Logger.recordOutput("Elapsed Time", latestShiftInfo.elapsedTime());
        Logger.recordOutput("Remaining Time", latestShiftInfo.remainingTime());
        Logger.recordOutput("Shift Active", latestShiftInfo.active());
    }

    @Override
    public void disabledInit() {
        HubShiftUtil.initialize();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        HubShiftUtil.initialize();
        vision.disableVision();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        HubShiftUtil.initialize();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {}

    @Override
    public void utilityInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void utilityPeriodic() {}

    @Override
    public void utilityExit() {}

    @Override
    public void simulationPeriodic() {}
}
