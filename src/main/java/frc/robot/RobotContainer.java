// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.


package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.AlgaeIntakeSubsystem;
import frc.robot.subsystems.swervedrive.CoralIntakeSubsystem;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem;
// import frc.robot.subsystems.swervedrive.ElevatorSubsystemBinary;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystemTest;
// import frc.robot.subsystems.swervedrive.ElevatorTest;
// import java.io.File;
import swervelib.SwerveInputStream;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        // Replace with CommandPS4Controller or CommandJoystick if needed
        final         CommandXboxController driverXbox = new CommandXboxController(0);
        // The robot's subsystems and commands are defined here...
        private final SwerveSubsystemTest testSwerve = new SwerveSubsystemTest(new File(Filesystem.getDeployDirectory(), "swerve"));
        // private final SwerveSubsystem       drivebase  = new SwerveSubsystem();
        private final AlgaeIntakeSubsystem algaeIntake = new AlgaeIntakeSubsystem();
        private final CoralIntakeSubsystem coralIntake = new CoralIntakeSubsystem();
        // private final ElevatorSubsystemBinary elevatorBinary = new ElevatorSubsystemBinary();
        private final ElevatorSubsystem elevator = new ElevatorSubsystem();

        // private final ElevatorTest elevatorTest = new ElevatorTest();
        SendableChooser<Command> m_chooser = new SendableChooser<>();

        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(testSwerve.getSwerveDrive(),
                                                          () -> driverXbox.getLeftY() * 1,
                                                          () -> driverXbox.getLeftX() * 1)
                                                        .withControllerRotationAxis(driverXbox::getRightX)                                           
                                                        .deadband(OperatorConstants.DEADBAND)
                                                        .scaleTranslation(0.8)
                                                        .allianceRelativeControl(true);
                                                        // TODO: switched alliance relative control -> should test before match

        public RobotContainer() {
                // Configure the trigger bindings
                configureButtonBindings();

                testSwerve.zeroGyro();
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary
         * predicate, or via the named factories in {@link
         * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * {@link
         * CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or
         * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * joysticks}.
         */

        // private void configureHeading() {
        //   SwerveSubsystem.zeroGyro;
        // }

        private void configureButtonBindings() {
                Command driveFieldOrientedAnglularVelocity = testSwerve.driveFieldOriented(driveAngularVelocity);

                testSwerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
              
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous

                return new PathPlannerAuto("auto1");

                // return new PathPlannerAuto("CoralAuto");
        }

        


}



// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import java.io.File;

// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
// // import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Constants.ButtonConstants;
// import frc.robot.Constants.ElevatorConstants;
// import frc.robot.Constants.OperatorConstants;
// import frc.robot.subsystems.swervedrive.AlgaeIntakeSubsystem;
// import frc.robot.subsystems.swervedrive.CoralIntakeSubsystem;
// import frc.robot.subsystems.swervedrive.ElevatorSubsystem;
// // import frc.robot.subsystems.swervedrive.ElevatorSubsystemBinary;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;
// import frc.robot.subsystems.swervedrive.SwerveSubsystemTest;
// // import frc.robot.subsystems.swervedrive.ElevatorTest;
// // import java.io.File;
// import swervelib.SwerveInputStream;

// /**
//  * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
//  * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
//  * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
//  */
// public class RobotContainer
// {

//   // Replace with CommandPS4Controller or CommandJoystick if needed
//   final         CommandXboxController driverXbox = new CommandXboxController(0);
//   // The robot's subsystems and commands are defined here...
//   private final SwerveSubsystemTest testSwerve = new SwerveSubsystemTest(new File(Filesystem.getDeployDirectory(), "swerve"));
//   private final SwerveSubsystem       drivebase  = new SwerveSubsystem();
//   private final AlgaeIntakeSubsystem algaeIntake = new AlgaeIntakeSubsystem();
//   private final CoralIntakeSubsystem coralIntake = new CoralIntakeSubsystem();
//   // private final ElevatorSubsystemBinary elevatorBinary = new ElevatorSubsystemBinary();
//   private final ElevatorSubsystem elevator = new ElevatorSubsystem();

//   // private final ElevatorTest elevatorTest = new ElevatorTest();

//   /**
//    * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
//    */
//   /* 
//   SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
//                                                                 () -> driverXbox.getLeftY() * 1,
//                                                                 () -> driverXbox.getLeftX() * 1)
//                                                             .withControllerRotationAxis(driverXbox::getRightX)                                           
//                                                             .deadband(OperatorConstants.DEADBAND)
//                                                             .scaleTranslation(0.8)
//                                                             .allianceRelativeControl(true);
//                                                             // TODO: switched alliance relative control -> should test before match

//   */

//   SwerveInputStream driveAngularVelocity = SwerveInputStream.of(testSwerve.getSwerveDrive(),
//                                                             () -> driverXbox.getLeftY() * 1,
//                                                             () -> driverXbox.getLeftX() * 1)
//                                                         .withControllerRotationAxis(driverXbox::getRightX)                                           
//                                                         .deadband(OperatorConstants.DEADBAND)
//                                                         .scaleTranslation(0.8)
//                                                         .allianceRelativeControl(true);
//                                                         // TODO: switched alliance relative control -> should test before match

  

//   /**
//    * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
//    */
//   SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
//                                                                                              driverXbox::getRightY)
//                                                            .headingWhile(true);

//   /**
//    * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
//    */
//   SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
//                                                              .allianceRelativeControl(false);

//   SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
//                                                                         () -> -driverXbox.getLeftY(),
//                                                                         () -> -driverXbox.getLeftX() * -1)
//                                                                     .withControllerRotationAxis(driverXbox::getRightX)
//                                                                     .deadband(OperatorConstants.DEADBAND)
//                                                                     .scaleTranslation(0.8)
//                                                                     .allianceRelativeControl(true);
//   // Derive the heading axis with math!
//   SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
//                                                                                .withControllerHeadingAxis(() ->
//                                                                                                               Math.sin(
//                                                                                                                   driverXbox.getRawAxis(
//                                                                                                                       2) *
//                                                                                                                   Math.PI) *
//                                                                                                               (Math.PI *
//                                                                                                                2),
//                                                                                                           () ->
//                                                                                                               Math.cos(
//                                                                                                                   driverXbox.getRawAxis(
//                                                                                                                       2) *
//                                                                                                                   Math.PI) *
//                                                                                                               (Math.PI *
//                                                                                                                2))
//                                                                                .headingWhile(true);

//   /**
//    * The container for the robot. Contains subsystems, OI devices, and commands.
//    */
//   public RobotContainer() {
//     NamedCommands.registerCommand("coralOuttake", coralIntake.setCoralIntakeRoller(Constants.IntakeConstants.CoralOuttakeSpeeds));
//     NamedCommands.registerCommand("coralOuttakeStop", coralIntake.setCoralIntakeRoller(0.0));

//     algaeIntake.setDefaultCommand(algaeIntake.setAlgaeIntakeRoller(0));
//     coralIntake.setDefaultCommand(coralIntake.setCoralIntakeRoller(0));

//     // Command [name] = new subsystem.command();

//     // do L1, coral outtake, have param = Constants.IntakeConstants.CoralOuttakeSpeeds
//     // have another command to stop elevator

//     // like below, NamedCommands.registerCommand("name", Commands.something());

    
//     // Configure the trigger bindings
//     configureBindings();
//     DriverStation.silenceJoystickConnectionWarning(true);
//     NamedCommands.registerCommand("test", Commands.print("I EXIST"));

//   }
  


//   /**
//    * Use this method to define your trigger->command mappings. Triggers can be created via the
//    * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
//    * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
//    * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
//    * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
//    */
//   private void configureBindings()
//   {

//     // Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
//     Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
//     // Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
//     //    driveDirectAngle);
//     // Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
//     // Command driveFieldOrientedAngularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
//     // Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
//     //    driveDirectAngleKeyboard);

//     if (RobotBase.isSimulation())
//     {
//       drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
      

//     } else
//     {
//       drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
//       // driverXbox.button(ButtonConstants.xboxX).whileTrue(algaeIntake.setAlgaeIntakeRoller(Constants.IntakeConstants.AlgaeIntakeSpeeds));
//       // driverXbox.button(ButtonConstants.xboxB).whileTrue(algaeIntake.setAlgaeIntakeRoller(Constants.IntakeConstants.AlgaeOuttakeSpeeds));
      
//       driverXbox.button(ButtonConstants.xboxY).onTrue(elevator.moveTo(ElevatorConstants.vL4Height));
//       driverXbox.button(ButtonConstants.xboxX).onTrue(elevator.moveTo(ElevatorConstants.vL3Height));
//       driverXbox.button(ButtonConstants.xboxB).onTrue(elevator.moveTo(ElevatorConstants.vL2Height));
//       driverXbox.button(ButtonConstants.xboxA).onTrue(elevator.moveTo(ElevatorConstants.vL1Height));

//       driverXbox.button(ButtonConstants.xboxRB).whileTrue(coralIntake.setCoralIntakeRoller(Constants.IntakeConstants.CoralIntakeSpeeds));
//       driverXbox.axisGreaterThan(3, 0.2).whileTrue(coralIntake.setCoralIntakeRoller(Constants.IntakeConstants.CoralOuttakeSpeeds));
//       // right bumper greater than 0.2 = go on coral scoring

//       driverXbox.button(ButtonConstants.xboxRB).whileTrue(algaeIntake.setAlgaeIntakeRoller(Constants.IntakeConstants.AlgaeIntakeSpeeds));
//       driverXbox.axisGreaterThan(2, 0.2).whileTrue(algaeIntake.setAlgaeIntakeRoller(Constants.IntakeConstants.AlgaeOuttakeSpeeds));
//       // left bumper greater than 0.2 = go on algae scoring

//       driverXbox.pov(180).onTrue(new InstantCommand(() -> drivebase.zeroGyro()));
//     }

//     if (Robot.isSimulation())
//     {
//       driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
//       driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

//     }
//     if (DriverStation.isTest())
//     {
//       drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity); // Overrides drive command above!

//       driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
//       driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
//       driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
//       driverXbox.back().whileTrue(drivebase.centerModulesCommand());
//       driverXbox.leftBumper().onTrue(Commands.none());
//       driverXbox.rightBumper().onTrue(Commands.none());
      
//     } else
//     {
//       /* 
//       driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
//       driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
//       driverXbox.b().whileTrue(
//           drivebase.driveToPose(
//               new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                               );
//       driverXbox.start().whileTrue(Commands.none());
//       driverXbox.back().whileTrue(Commands.none());
//       driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
//       driverXbox.rightBumper().onTrue(Commands.none());
//       */
//     }

//   }

//   /**
//    * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */
//   public Command getAutonomousCommand()
//   {
//     // An example command will be run in autonomous
//     // return drivebase.getAutonomousCommand("New Auto");
//     return new PathPlannerAuto("Auto1");
//   }

//   public void setMotorBrake(boolean brake)
//   {
//     drivebase.setMotorBrake(brake);
//   }
// }
