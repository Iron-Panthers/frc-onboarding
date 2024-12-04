// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.Elevator.ElevatorVelocityTarget;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsIO;
import frc.robot.subsystems.flywheels.Flywheels.VelocityTarget;
import frc.robot.subsystems.flywheels.FlywheelsIOTalonFX;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.Rollers.RollerState;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.intake.IntakeIOTalonFX;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOTalonFX;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController driverA = new CommandXboxController(0);
  private final CommandXboxController driverB = new CommandXboxController(1);

  private Drive swerve; // FIXME make final, implement other robot types
  private Rollers rollers;
  private Flywheels flywheels;
  private Elevator elevator;

  public RobotContainer() {
    Intake intake = null;

    if (Constants.getRobotMode() != Mode.REPLAY) {
      switch (Constants.getRobotType()) {
        case COMP -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[3]));
          intake = new Intake(new IntakeIOTalonFX());
          flywheels = new Flywheels(new FlywheelsIOTalonFX());
          elevator = new Elevator(new ElevatorIOTalonFX());
        }
        case DEV -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[3]));
          intake = new Intake(new IntakeIOTalonFX()); // FIXME
          flywheels = new Flywheels(new FlywheelsIOTalonFX());
          elevator = new Elevator(new ElevatorIOTalonFX());
        }
        case SIM -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[3]));
          intake = new Intake(new IntakeIOTalonFX()); // FIXME
          flywheels = new Flywheels(new FlywheelsIOTalonFX());
          elevator = new Elevator(new ElevatorIOTalonFX());
        }
      }
    }

    if (swerve == null) {
      swerve =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }

    rollers = new Rollers(intake);

    configureBindings();
    configureAutos();
  }

  private void configureBindings() {
    // -----Driver Controls-----
    /*swerve.setDefaultCommand(
    swerve
        .run(
            () -> {
              swerve.driveTeleopController(
                  -driverA.getLeftY(), -driverA.getLeftX(), -driverA.getRightX());
            })
        .withName("Drive Teleop"));*/

    // -----Intake Controls-----
    driverA.x().whileTrue(rollers.setTargetCommand(RollerState.INTAKE));

    // -----Flywheel Controls-----
    //
    driverA
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  flywheels.setVelocityTarget(VelocityTarget.SHOOT);
                },
                flywheels));
    driverA
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  flywheels.setVelocityTarget(VelocityTarget.SLOW);
                },
                flywheels));
    driverA
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  flywheels.setVelocityTarget(VelocityTarget.IDLE);
                },
                flywheels));

// -----Elevator Controls-----
//
    driverA
        .rightTrigger()
        .onTrue(
            new InstantCommand(
              () -> {
                elevator.setVelocityTarget(ElevatorVelocityTarget.UP);
              },
              elevator));
    driverA
        .leftTrigger()
        .onTrue(
          new InstantCommand(
            () -> {
              elevator.setVelocityTarget(ElevatorVelocityTarget.DOWN);
            },
            elevator));
  }

  private void configureAutos() {}
}
