// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.Optional;

import org.opencv.dnn.Net;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */


  public Limelight() {}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double x = LimelightHelpers.getTA("limelight-front");
    double y = LimelightHelpers.getTX("limelight-front");
    SmartDashboard.putNumber("x", x);
    SmartDashboard.putNumber("y", y);
    this.Front_tag_id();
  }

  public double Limelight_front_y() { //returns robot forward speed based on the front LL

    double x = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("ta").getDouble(0);
    double x_speed = 0.3;
    if (x != 0) {
      x_speed = 1 / x;
    }

    if (x >= 18) {
      x_speed = 0.3;
    }

    x_speed *= -0.85;

    return x_speed;
  }

  public double Limelight_front_x() { // returns the robot side-way speed based on the front LL
    double y_speed = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tx").getDouble(0);
    y_speed *= -0.017;
    return y_speed;
  }

  public double Front_tag_id() {
    double id = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tid").getDouble(0);
    SmartDashboard.putNumber("id", id);
    return id;
  }

}
