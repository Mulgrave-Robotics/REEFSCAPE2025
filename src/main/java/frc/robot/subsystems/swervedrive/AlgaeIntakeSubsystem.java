package frc.robot.subsystems.swervedrive;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase
{

  private final SparkMax upperMotor = new SparkMax(IntakeConstants.algaeUpperMotorID, MotorType.kBrushless);

  public AlgaeIntakeSubsystem()
  {

  }

  public Command setAlgaeIntakeRoller(double speed)
  {
    return run(() -> {
      upperMotor.set(speed);
    });
  }
}