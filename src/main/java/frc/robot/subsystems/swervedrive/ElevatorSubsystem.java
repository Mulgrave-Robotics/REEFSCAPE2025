package frc.robot.subsystems.swervedrive;

// import com.revrobotics.AbsoluteEncoder;
//import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkFlex upperMotor;
    private final SparkFlex lowerMotor;
    private final RelativeEncoder encoder;
    private double currentHeight;

    public ElevatorSubsystem() {
        upperMotor = new SparkFlex(ElevatorConstants.elevatorUpperMotorID, MotorType.kBrushless);
        lowerMotor = new SparkFlex(ElevatorConstants.elevatorLowerMotorID, MotorType.kBrushless);

        currentHeight = 0;

        // ✅ Motor Configuration
        SparkFlexConfig upperMotorConfig = new SparkFlexConfig();
        SparkFlexConfig lowerMotorConfig = new SparkFlexConfig();

        upperMotorConfig
                .smartCurrentLimit(40)  // Reduced from 50A to prevent brownouts
                .idleMode(IdleMode.kBrake);

        lowerMotorConfig
                .apply(upperMotorConfig)
                .idleMode(IdleMode.kBrake)
                .inverted(true);

        // ✅ Apply configurations
        upperMotor.configure(upperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lowerMotor.configure(lowerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        // ✅ Encoder setup
        encoder = upperMotor.getEncoder();
        // get positive direction motor encoder
        // positive one should be the motor that is not sucky
        encoder.setPosition(0);
    }

    public double getPositionInches() {
        // 0.9848 is sin(79.99), converts slanted height into vertical height
        return (0.9848 * encoder.getPosition() * (2 * Math.PI * ElevatorConstants.kGearRatio));           
    }
                           
    public void reachLevel(double targetHeight){

        currentHeight = getPositionInches();
        
        if (targetHeight > currentHeight)
        {

            upperMotor.set(ElevatorConstants.kMaxSpeedPercentage * 1.0);
            // positive
            //lowerMotor.set(ElevatorConstants.kMaxSpeedPercentage * 1.0);

        }

        else if (targetHeight < currentHeight)
        {
            upperMotor.set(ElevatorConstants.kMaxSpeedPercentage * -1.0);
            // negative
            //lowerMotor.set(ElevatorConstants.kMaxSpeedPercentage * -1.0);
        }

        else 
        {
            SmartDashboard.putString("elevator height status", "Elevator is already at wanted height!");
        }

        SmartDashboard.putNumber("currentHeight", currentHeight);
        
    }

    public Command setLevel(double targetHeight)
    {
        return run(() -> reachLevel(targetHeight));
    }

    public Command moveTo(double targetHeight)
    {
        SmartDashboard.putNumber("targetHeight", targetHeight);

        return setLevel(targetHeight)
        .until(() -> aroundHeight(targetHeight))
        .andThen(() -> upperMotor.set(0.0));

    }

    public boolean aroundHeight(double height)
    {
        return aroundHeight(height, ElevatorConstants.kElevatorDefaultTolerance);
    }
    public boolean aroundHeight(double height, double tolerance)
    {
        return MathUtil.isNear(height,getPositionInches(),tolerance);
    }
}
