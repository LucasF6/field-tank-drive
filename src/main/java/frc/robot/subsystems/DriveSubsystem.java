// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Drive.*;

import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase {
  CANSparkMax m_frontRightMotor, m_frontLeftMotor, m_backRightMotor, m_backLeftMotor;
  RelativeEncoder m_rightEncoder, m_leftEncoder;
  WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(GYRO_ID);
  DifferentialDrive m_diffDrive;

  PIDController m_turningPID = new PIDController(TURNING_kP, TURNING_kI, TURNING_kD);
  DifferentialDriveOdometry m_odometry;
  SlewRateLimiter m_accLimiter = new SlewRateLimiter(ACCELERATION_LIMIT);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_frontRightMotor = new CANSparkMax(FRONT_RIGHT_ID, MotorType.kBrushless);
    m_frontLeftMotor = new CANSparkMax(FRONT_LEFT_ID, MotorType.kBrushless);
    m_backRightMotor = new CANSparkMax(BACK_RIGHT_ID, MotorType.kBrushless);
    m_backLeftMotor = new CANSparkMax(BACK_LEFT_ID, MotorType.kBrushless);
    
    m_frontRightMotor.setInverted(true);
    m_backRightMotor.setInverted(true);
    m_backLeftMotor.follow(m_frontLeftMotor);
    m_backRightMotor.follow(m_frontRightMotor);

    m_diffDrive = new DifferentialDrive(m_frontLeftMotor, m_frontRightMotor);
    
    m_rightEncoder = m_frontRightMotor.getEncoder();
    m_leftEncoder = m_frontLeftMotor.getEncoder();
    m_rightEncoder.setPositionConversionFactor(POSITION_CONVERSION);
    m_leftEncoder.setPositionConversionFactor(POSITION_CONVERSION);
    m_rightEncoder.setVelocityConversionFactor(VELOCITY_CONVERSION);
    m_leftEncoder.setVelocityConversionFactor(VELOCITY_CONVERSION);
    m_rightEncoder.setPosition(0);
    m_leftEncoder.setPosition(0);

    m_turningPID.enableContinuousInput(0, Math.PI); // Forward and backward are the same position
    m_turningPID.setTolerance(ANGLE_TOLERANCE);
    m_gyro.reset();
    m_odometry = new DifferentialDriveOdometry(
        m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    m_accLimiter.reset(0);
  }

  @Override
  public void periodic() {
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
  }

  public double getAngle() {
    return Units.degreesToRadians(m_gyro.getAngle());
  }

  public void reset() {
    m_gyro.reset();
    m_accLimiter.reset(0);
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), 
        m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), new Pose2d());
  }

  public void fieldRelativeDrive(double speed, double rotation) {
    double gyroAngle = Units.degreesToRadians(m_gyro.getAngle());
    double twist = MathUtil.clamp(m_turningPID.calculate(gyroAngle, rotation),
        -MAX_TURN, MAX_TURN);
    speed *= MAX_SPEED;
    speed *= Math.abs(gyroAngle - rotation) < Math.PI ? 1 : -1;
    if (m_turningPID.atSetpoint()) {
      speed = m_accLimiter.calculate(speed);
    } else {
      speed = m_accLimiter.calculate(speed / 3);
    }
    m_diffDrive.arcadeDrive(speed, twist);
  }

  public void robotRelativeDrive(double speed, double rotation) {
    speed = m_accLimiter.calculate(speed);
    rotation *= MAX_TURN;
    m_diffDrive.arcadeDrive(speed, rotation);
  }

  public Command getResetCommand() {
    return runOnce(this::reset);
  }

  public Command getFieldRelativeDriveCommand(DoubleSupplier speed, DoubleSupplier rotation) {
    return run(() -> fieldRelativeDrive(speed.getAsDouble(), rotation.getAsDouble()));
  }

  public Command getRobotRelativeDriveCommand(DoubleSupplier speed, DoubleSupplier rotation) {
    return run(() -> robotRelativeDrive(speed.getAsDouble(), rotation.getAsDouble()));
  }
}
