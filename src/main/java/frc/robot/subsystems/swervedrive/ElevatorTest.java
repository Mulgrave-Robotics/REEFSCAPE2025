// package frc.robot.subsystems.swervedrive;

// import com.fasterxml.jackson.databind.PropertyNamingStrategies.LowerCamelCaseStrategy;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkFlexConfig;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ElevatorConstants;

// public class ElevatorTest extends SubsystemBase{
//     private final SparkFlex upperMotor;
//     private final SparkFlex lowerMotor;
//     private final RelativeEncoder encoder;

//     public ElevatorTest() {
//         upperMotor = new SparkFlex(ElevatorConstants.elevatorUpperMotorID, MotorType.kBrushless);
//         lowerMotor = new SparkFlex(ElevatorConstants.elevatorLowerMotorID, MotorType.kBrushless);

//         // ✅ Motor Configuration
//         SparkFlexConfig upperMotorConfig = new SparkFlexConfig();
//         SparkFlexConfig lowerMotorConfig = new SparkFlexConfig();

//         upperMotorConfig
//                 .smartCurrentLimit(40)  // Reduced from 50A to prevent brownouts
//                 .idleMode(IdleMode.kBrake);

//         lowerMotorConfig
//                 .smartCurrentLimit(40)  // Reduced from 50A to prevent brownouts
//                 .idleMode(IdleMode.kBrake);
//                     //.inverted(true);

//         // ✅ Apply configurations
//         upperMotor.configure(upperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         lowerMotor.configure(lowerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


//         // ✅ Encoder setup
//         encoder = upperMotor.getEncoder();
//         // get positive direction motor encoder
//         // positive one should be the motor that is not sucky
//         encoder.setPosition(0);
//     }

//     public Command testUp10(){
//         return run(()-> {
//             upperMotor.set(ElevatorConstants.kMaxSpeedPercentage * 1.0);
//         });

//     }

//     public Command testDown10(){
//         return run(()-> {
//             upperMotor.set(ElevatorConstants.kMaxSpeedPercentage * -1.0);
//         });

//     }

//     public Command testUp9(){
//         return run(()-> {
//             lowerMotor.set(ElevatorConstants.kMaxSpeedPercentage * 1.0);
//         });

//     }

//     public Command testDown9(){
//         return run(()-> {
//             lowerMotor.set(ElevatorConstants.kMaxSpeedPercentage * -1.0);
//         });

//     }

// }
