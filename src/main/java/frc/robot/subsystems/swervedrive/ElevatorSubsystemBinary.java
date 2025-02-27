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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystemBinary extends SubsystemBase {
    private final SparkFlex upperMotor;
    private final SparkFlex lowerMotor;
    private final RelativeEncoder encoder;
    private int currentLevel;

    public ElevatorSubsystemBinary() {
        upperMotor = new SparkFlex(ElevatorConstants.elevatorUpperMotorID, MotorType.kBrushless);
        lowerMotor = new SparkFlex(ElevatorConstants.elevatorLowerMotorID, MotorType.kBrushless);

        // ✅ Motor Configuration
        SparkFlexConfig upperMotorConfig = new SparkFlexConfig();
        SparkFlexConfig lowerMotorConfig = new SparkFlexConfig();


        upperMotorConfig
                .smartCurrentLimit(50)
                .idleMode(IdleMode.kBrake);

        lowerMotorConfig
                .apply(upperMotorConfig)
                .inverted(true);

        // ✅ Apply configurations
        upperMotor.configure(upperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lowerMotor.configure(lowerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ✅ Encoder
        encoder = lowerMotor.getEncoder();
        encoder.setPosition(0);
        currentLevel = 1;
    }

    //Lok's approach

    public double getPositionInches() {
        return encoder.getPosition() * (2 * Math.PI * ElevatorConstants.kGearRatio);           
    }

    public void reachLevel(int targetLevel, int direction){
        
        if (direction > 0) {
            // move up
            if (targetLevel <= ElevatorConstants.kMaxLevel) {
                upperMotor.set(ElevatorConstants.kMaxSpeedPercentage * direction);
                lowerMotor.set(ElevatorConstants.kMaxSpeedPercentage * direction);

                currentLevel++;
                SmartDashboard.putNumber("currentLevel", currentLevel);
            }
        } else {
            // move down
            if (targetLevel >= ElevatorConstants.kMinLevel) {
                upperMotor.set(ElevatorConstants.kMaxSpeedPercentage * direction);
                lowerMotor.set(ElevatorConstants.kMaxSpeedPercentage * direction);

                currentLevel--;
                SmartDashboard.putNumber("currentLevel", currentLevel);
            }

        }
        
    }

    public Command setLevel(int targetLevel, int direction){
        return run(()-> reachLevel(targetLevel, direction));
    }
    
    // Move one level up
    public Command moveUp() {
        double targetHeight;

        if (currentLevel == 1) 
        {
            targetHeight = ElevatorConstants.eL2Height;
        } 
        else if (currentLevel == 2) 
        {
            targetHeight = ElevatorConstants.eL3Height;
        } 
        else if (currentLevel == 3) 
        {
            targetHeight = ElevatorConstants.eL4Height;
        } 
        else 
        {
            SmartDashboard.putString("error", "error, already highest level");
            return Commands.none(); // Do nothing if at max level
        }

        SmartDashboard.putNumber("targetHeight", targetHeight);
        SmartDashboard.putNumber("targetLevel", currentLevel + 1);

        return setLevel(currentLevel + 1, 1)
            .until(() -> aroundHeight(targetHeight))
            .andThen(() -> lowerMotor.set(0.0));
    }


    // Move one level down
    public Command moveDown(){
        double targetHeight;

        if (currentLevel == 1) 
        {
            targetHeight = ElevatorConstants.eL2Height;
        } 
        else if (currentLevel == 2) 
        {
            targetHeight = ElevatorConstants.eL3Height;
        } 
        else if (currentLevel == 3) 
        {
            targetHeight = ElevatorConstants.eL4Height;
        } 
        else 
        {
            SmartDashboard.putString("error", "error, already highest level");
            return Commands.none(); // Do nothing if at max level
        }

        SmartDashboard.putNumber("targetHeight", targetHeight);
        SmartDashboard.putNumber("targetLevel", currentLevel-1);

        return setLevel((currentLevel - 1), (-1))
            .until(()->aroundHeight(targetHeight))
            .andThen(() -> lowerMotor.set(0.0));
    }

    public boolean aroundHeight(double height){
        return aroundHeight(height, ElevatorConstants.kElevatorDefaultTolerance);
    }
    public boolean aroundHeight(double height, double tolerance){
        return MathUtil.isNear(height,getPositionInches(),tolerance);
    }
}