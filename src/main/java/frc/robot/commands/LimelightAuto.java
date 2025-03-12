// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LimelightAuto extends Command {
  private Limelight limelight;
  private SwerveSubsystem swerveSubsystem;

  private double Limelight_front_x;
  private double Limelight_front_y;

  /** Creates a new limelightAuto. */
  public LimelightAuto(Limelight limelight, SwerveSubsystem swerveSubsystem) {
    this.limelight = limelight;
    this.swerveSubsystem = swerveSubsystem;

    addRequirements(limelight);
    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Limelight_front_x < 22 && Limelight_front_x != 0) {
      Limelight_front_x = limelight.Limelight_front_x();
      Limelight_front_y = limelight.Limelight_front_y();

      swerveSubsystem.driveCommand(() -> Limelight_front_x, () -> Limelight_front_y, () -> 0);
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
