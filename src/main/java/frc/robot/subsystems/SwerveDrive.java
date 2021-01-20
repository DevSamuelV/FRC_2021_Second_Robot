// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveDrive extends SubsystemBase {
  private CANSparkMax frontLeftMotion = new CANSparkMax(Constants.frontLeftMotorCANID, MotorType.kBrushless);
  private CANSparkMax frontRightMotion = new CANSparkMax(Constants.frontRightMotorCANID, MotorType.kBrushless);
  private CANSparkMax backLeftMotion = new CANSparkMax(Constants.backLeftMotorCANID, MotorType.kBrushless);
  private CANSparkMax backRightMotion = new CANSparkMax(Constants.backRightMotorCANID, MotorType.kBrushless);

  private CANSparkMax frontLeftAngle = new CANSparkMax(Constants.frontLeftMotorCANID, MotorType.kBrushless);
  private CANSparkMax frontRightAngle = new CANSparkMax(Constants.frontRightMotorCANID, MotorType.kBrushless);
  private CANSparkMax backLeftAngle = new CANSparkMax(Constants.backLeftMotorCANID, MotorType.kBrushless);
  private CANSparkMax backRightAngle = new CANSparkMax(Constants.backRightMotorCANID, MotorType.kBrushless);

  // private PIDController pid = new PIDController(1, 0, 0);

  /** Creates a new DriveTrain. */
  public SwerveDrive() {

  }

  @Override
  public void periodic() {
  }

  public void DriveManual(double x1, double x2, double y1) {
    double r = Math.sqrt((Constants.HIGHT * Constants.HIGHT) + (Constants.WIDTH * Constants.WIDTH));

    y1 *= -1;

    double a = x1 - x2 * (Constants.HIGHT / r);
    double b = x1 + x2 * (Constants.HIGHT / r);
    double c = y1 - x2 * (Constants.WIDTH / r);
    double d = y1 + x2 * (Constants.WIDTH / r);

    double backLeftSpeed = Math.sqrt((a * a) + (b * b));
    double backRightSpeed = Math.sqrt((a * a) + (c * c));
    double frontRightSpeed = Math.sqrt((b * b) + (d * d));
    double frontLeftSpeed = Math.sqrt((b * b) + (c * c));

    double backLeftAngle = Math.atan2(a, d) / Math.PI;
    double backRightAngle = Math.atan2(a, c) / Math.PI;
    double frontLeftAngle = Math.atan2(b, d) / Math.PI;
    double frontRightAngle = Math.atan2(b, c) / Math.PI;

    this.frontRightMotion.set(frontRightSpeed);
    this.frontLeftMotion.set(frontLeftSpeed);
    this.backLeftMotion.set(backLeftSpeed);
    this.backRightMotion.set(backRightSpeed);

    this.frontRightAngle.set(frontRightAngle);
    this.frontLeftAngle.set(frontLeftAngle);
    this.backLeftAngle.set(backLeftAngle);
    this.backRightAngle.set(backRightAngle);

    SmartDashboard.putNumber("Back Left Speed", backLeftSpeed);
    SmartDashboard.putNumber("Back Right Speed", backRightSpeed);
    SmartDashboard.putNumber("Front Right Speed", frontRightSpeed);
    SmartDashboard.putNumber("Front Left Speed", frontLeftSpeed);

    SmartDashboard.putNumber("Back Left Angle", backLeftAngle);
    SmartDashboard.putNumber("Back Right Angle", backRightAngle);
    SmartDashboard.putNumber("Front Left Angle", frontLeftAngle);
    SmartDashboard.putNumber("Front Right Angle", frontRightAngle);
  }
}
