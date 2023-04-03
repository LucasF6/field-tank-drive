// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.Drive.*;

public class TurnAroundCommand extends CommandBase {
  DriveSubsystem m_driveSubsystem;
  PIDController m_turningPID = new PIDController(TURNING_kP, TURNING_kI, TURNING_kD);

  /** Creates a new TurnAroundCommand. */
  public TurnAroundCommand(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_turningPID.enableContinuousInput(0, 2 * Math.PI);
    m_turningPID.setTolerance(ANGLE_TOLERANCE);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turningPID.setSetpoint(m_driveSubsystem.getAngle() + Math.PI);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.robotRelativeDrive(
        0, m_turningPID.calculate(m_driveSubsystem.getAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_turningPID.atSetpoint();
  }
}
