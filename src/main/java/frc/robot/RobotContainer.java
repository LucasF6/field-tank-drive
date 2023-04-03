// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.TurnAroundCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private final CommandJoystick m_joystick = new CommandJoystick(0);

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  private final Command m_resetDriveCommand = m_driveSubsystem.getResetCommand();
  private final Command m_fieldRelativeDriveCommand = 
      m_driveSubsystem.getFieldRelativeDriveCommand(
          m_joystick::getMagnitude,
          m_joystick::getDirectionRadians);
  private final Command m_robotRelativeDriveCommand =
      m_driveSubsystem.getRobotRelativeDriveCommand(
          m_joystick::getY,
          m_joystick::getX);
  private final Command m_turnAroundCommand = new TurnAroundCommand(m_driveSubsystem);

  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(m_fieldRelativeDriveCommand);
    configureBindings();
  }

  private void configureBindings() {
    m_joystick.trigger().onTrue(m_turnAroundCommand);
    m_joystick.top().toggleOnTrue(m_robotRelativeDriveCommand);
    m_joystick.povDown().onTrue(m_resetDriveCommand);
  }

  public Command getAutonomousCommand() {
    return null;
  }

  public Command getTeleopCommand() {
    return null;
  }
}
