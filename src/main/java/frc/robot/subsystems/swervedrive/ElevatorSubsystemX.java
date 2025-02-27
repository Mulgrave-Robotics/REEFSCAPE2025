// package frc.robot.subsystems.swervedrive;

// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkFlexConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.RelativeEncoder;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ElevatorConstants;

// public class ElevatorSubsystemX extends SubsystemBase {
//     private final SparkFlex upperMotor;
//     private final SparkFlex lowerMotor;
//     private final RelativeEncoder encoder;
//     private int currentLevel;
//     private int targetLevel;

//     public ElevatorSubsystemX() {
//         upperMotor = new SparkFlex(ElevatorConstants.elevatorUpperMotorID, MotorType.kBrushless);
//         lowerMotor = new SparkFlex(ElevatorConstants.elevatorLowerMotorID, MotorType.kBrushless);

//         // ✅ Motor Configuration
//         SparkFlexConfig upperMotorConfig = new SparkFlexConfig();
//         SparkFlexConfig lowerMotorConfig = new SparkFlexConfig();

//         upperMotorConfig
//                 .smartCurrentLimit(40)  // Reduced from 50A to prevent brownouts
//                 .idleMode(IdleMode.kBrake);

//         lowerMotorConfig
//                 .apply(upperMotorConfig)
//                 .inverted(true);

//         // ✅ Apply configurations
//         upperMotor.configure(upperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         lowerMotor.configure(lowerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


//         // ✅ Encoder setup
//         encoder = lowerMotor.getEncoder();
//         encoder.setPosition(0);
//         currentLevel = 1;
//         targetLevel = 1;
//     }

//     public void syncMotors(double power) {
//         upperMotor.set(power);
//         lowerMotor.set(power);
//     }

//     public void setTargetLevel(int level) {
//         targetLevel = MathUtil.clamp(level, ElevatorConstants.kMinLevel, ElevatorConstants.kMaxLevel);
//         SmartDashboard.putNumber("Set Target Level", targetLevel);
//     }

//     public int getCurrentLevel() {
//         return currentLevel;
//     }

//     public int getTargetLevel() {
//         return targetLevel;
//     }

//     public double getLevelHeight(int level) {
//         switch (level) {
//             case 1: return ElevatorConstants.eL1Height;
//             case 2: return ElevatorConstants.eL2Height;
//             case 3: return ElevatorConstants.eL3Height;
//             case 4: return ElevatorConstants.eL4Height;
//             default: return 0;
//         }
//     }

//     public double getPositionInches() {
//         return (encoder.getPosition() * (2 * Math.PI * ElevatorConstants.kGearRatio)) + ElevatorConstants.kZeroOffset;
//     }

//     public void moveToTargetLevel() {
//         double targetHeight = getLevelHeight(targetLevel);
//         double speed = ElevatorConstants.kMaxSpeedPercentage;
    
//         if (currentLevel < targetLevel) {
//             syncMotors(speed);  // Move UP
//         } else if (currentLevel > targetLevel) {
//             syncMotors(-speed * 0.75);  // Move DOWN (slow down to prevent overshoot)
//         } else {
//             syncMotors(0.0);  // Stop when at target level
//         }
    
//         SmartDashboard.putNumber("Current Elevator Height", getPositionInches());
//         SmartDashboard.putNumber("Target Elevator Height", targetHeight);
//         SmartDashboard.putNumber("Current Level", currentLevel);
//     }
    

//     public boolean isAtTargetLevel() {
//         return MathUtil.isNear(getPositionInches(), getLevelHeight(targetLevel), ElevatorConstants.kElevatorDefaultTolerance);
//     }

//     public void stopMotors() {
//         lowerMotor.set(0.0);
//     }
// }
