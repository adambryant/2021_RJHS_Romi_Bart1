// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistance extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_distance;
  private final double m_speed;
  private PIDController pidController;

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistance(double speed, double inches, Drivetrain drive) {
    m_distance = inches;
    m_speed = speed;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
    m_drive.resetGyro();
    pidController = new PIDController(Constants.kP, Constants.kI, Constants.kD, 0.05);
    pidController.reset();
    pidController.setSetpoint(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
   public void execute() {
    //  m_drive.arcadeDrive(m_speed, getRotationEncoder());
     m_drive.arcadeDrive(m_speed, getRotationPidController());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    pidController.close();
    pidController.disableContinuousInput();
    pidController.reset();
    return Math.abs(m_drive.getAverageDistanceInch()) >= m_distance;
  }

  private double getRotationEncoder() {
    double left = m_drive.getLeftEncoderCount();
    double right = m_drive.getRightEncoderCount();
    double difference = left - right;
    double rotation = difference / 1000;
    //System.out.println(left + " " + right);
    return -rotation;
  }

  private double getRotationGyro() {
    double angle = m_drive.getGyroAngleZ();
    double rotation = angle / 180;
    System.out.println("angle: " + angle + " rotation: " + rotation);

    return -rotation;
  }

  private double getRotationPidController() {
    double angle = m_drive.getGyroAngleZ();
    double rotation = pidController.calculate(angle);
    System.out.println("angle: " + angle + " rotation: " + rotation);
    return rotation;
  }
}
