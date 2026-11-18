// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.MetersPerSecond;

import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandXboxController;
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
import frc.robot.subsystems.SubsystemManager.RobotState;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController joystick;

    public final Drivetrain drivetrain;
    public final Hood hood;
    public final Indexer indexer;
    public final IntakeRack intakeRack;
    public final IntakeRoller intakeRoller;
    public final RollerFloor rollerFloor;
    public final Shooter shooter;

    public final Autoaim autoaim;

    public final SubsystemManager subsystemManager;

    public RobotContainer(
        CommandXboxController joystick,
        Autoaim autoaim,
        SubsystemManager subsystemManager,
        Drivetrain drivetrain, 
        Hood hood, 
        Indexer indexer, 
        IntakeRack intakeRack, 
        IntakeRoller intakeRoller, 
        RollerFloor rollerFloor, 
        Shooter shooter) {

        this.joystick = joystick;

        this.autoaim = autoaim;

        this.subsystemManager = subsystemManager;

        this.drivetrain = drivetrain;
        this.hood = hood;
        this.indexer = indexer;
        this.intakeRack = intakeRack;
        this.intakeRoller = intakeRoller;
        this.rollerFloor = rollerFloor;
        this.shooter = shooter;
        
        configureBindings();
    }

    private void configureBindings() {    

        joystick.rightTrigger().onTrue(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING), subsystemManager)
        ).onFalse(
            Commands.either(            
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING), subsystemManager),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_IDLE), subsystemManager),
                joystick.leftTrigger()
            )
        );

        joystick.rightBumper().onTrue(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORINGMOVING), subsystemManager)
        ).onFalse(
            Commands.either(            
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING), subsystemManager),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_IDLE), subsystemManager),
                joystick.leftTrigger()
            )        
        );

        joystick.leftTrigger().onTrue(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING), subsystemManager)
        ).onFalse(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKEDEPLOYED), subsystemManager)
                .onlyIf(() -> !subsystemManager.isFiring())
        );

        joystick.leftBumper().onTrue(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_REVERSEINTAKING), subsystemManager)
        ).onFalse(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKEDEPLOYED), subsystemManager)
                .onlyIf(() -> !subsystemManager.isFiring())
        );

        joystick.dpadUp().onTrue(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING_TOWER_PRESET), subsystemManager)
        ).onFalse(
            Commands.either(            
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING), subsystemManager),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_IDLE), subsystemManager),
                joystick.leftTrigger()
            )        
        );

        joystick.dpadRight().onTrue(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING_TRENCH_PRESET), subsystemManager)
        ).onFalse(
            Commands.either(            
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING), subsystemManager),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_IDLE), subsystemManager),
                joystick.leftTrigger()
            )        
        );

        joystick.a().onTrue(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING_SYSTEMS_CHECK), subsystemManager)
        ).onFalse(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKEDEPLOYED), subsystemManager)
        );

        joystick.x().onTrue(Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.IDLE), subsystemManager));

        joystick.b().onTrue(Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_STOWED), subsystemManager));

        joystick.y().onTrue(Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_IDLE), subsystemManager));
       
        joystick.menu().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric).ignoringDisable(true));

        joystick.view().onTrue(Commands.runOnce(() -> intakeRack.zeroMotors()).ignoringDisable(true));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
}
