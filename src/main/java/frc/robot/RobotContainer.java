// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmJoystickCmd;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));
                                                                         
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IntakeSubsystem m_Intake = new IntakeSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final ClimberSubsystem m_climb = new ClimberSubsystem();
  private final LEDSubsystem m_LED = new LEDSubsystem(m_Intake);

  private final Command m_spinUpShooter = Commands.runOnce(m_shooter::enable, m_shooter);
  private final Command m_stopShooter = Commands.runOnce(m_shooter::disable, m_shooter);
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandPS5Controller driverController = new CommandPS5Controller(0);
   CommandPS5Controller m_operatorController =
      new CommandPS5Controller(OIConstants.kOperatorControllerPort);
  // private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Command> autoChooser;
    private final Field2d field;
    
      

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  //XboxController driverController = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
                                                          // Applies deadbands and inverts controls because joysticks
                                                          // are back-right positive while robot
                                                          // controls are front-left positive
                                                          () -> MathUtil.applyDeadband(driverController.getLeftY(),
                                                                                       OperatorConstants.LEFT_Y_DEADBAND),
                                                          () -> MathUtil.applyDeadband(driverController.getLeftX(),
                                                                                       OperatorConstants.LEFT_X_DEADBAND),
                                                          () -> -driverController.getRightX(),
                                                          () -> -driverController.getRightY());

    AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(drivebase,
                                                                         () ->
                                                                             MathUtil.applyDeadband(driverController.getLeftY(),
                                                                                                    OperatorConstants.LEFT_Y_DEADBAND),
                                                                         () -> MathUtil.applyDeadband(driverController.getLeftX(),
                                                                                                      OperatorConstants.LEFT_X_DEADBAND),
                                                                         () -> driverController.getRawAxis(2));

    // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
    //                                                                   () -> MathUtil.applyDeadband(driverController.getLeftY(),
    //                                                                                             OperatorConstants.LEFT_Y_DEADBAND),
    //                                                                   () -> MathUtil.applyDeadband(driverController.getLeftX(),
    //                                                                                               OperatorConstants.LEFT_X_DEADBAND),
    //                                                                   () -> MathUtil.applyDeadband(driverController.getRightX(),
    //                                                                                               OperatorConstants.RIGHT_X_DEADBAND), 
    //                                                                   driverController::getRawAxis, 
    //                                                                   driverController::getButtonPressed, 
    //                                                                   driverController::getXButtonPressed, 
    //                                                                   driverController::getBButtonPressed);

    TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
                                                    () -> MathUtil.applyDeadband(driverController.getLeftY(),
                                                                                 OperatorConstants.LEFT_Y_DEADBAND),
                                                    () -> MathUtil.applyDeadband(driverController.getLeftX(),
                                                                                 OperatorConstants.LEFT_X_DEADBAND),
                                                    () -> driverController.getRawAxis(2), () -> true);
    TeleopDrive closedFieldRel = new TeleopDrive(
        drivebase,
        () -> MathUtil.applyDeadband(-driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverController.getRawAxis(2), () -> true);
    TeleopDrive closedFieldRobotOriented = new TeleopDrive(
      drivebase,
        () -> MathUtil.applyDeadband(driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverController.getRawAxis(2), () -> false);

    drivebase.setDefaultCommand(closedFieldRel);
    m_arm.setDefaultCommand(new ArmJoystickCmd(m_arm, () -> MathUtil.applyDeadband(m_operatorController.getLeftY(), .05)));
    // Pathplanner commands
   NamedCommands.registerCommand("Arm to Subwoofer", Commands.run(() -> {
      m_arm.setMotor(ArmConstants.kSubwooferSpot);
      },
      m_arm).withTimeout(1.5)
    );
    NamedCommands.registerCommand("Arm to SubwooferSide", Commands.run(() -> {
      m_arm.setMotor(ArmConstants.kSubwooferSideSpot);
      },
      m_arm).withTimeout(2.5)
    );
     NamedCommands.registerCommand("Align Drivebase", new TeleopDrive( drivebase,
         () -> (0),
        () -> (0),
        () -> -drivebase.calculateTurnAngle(), () -> true).withTimeout(0.5));
    NamedCommands.registerCommand("Rev Shooter Wheels", Commands.run(m_shooter::setMotorFullSpeed, m_shooter).withTimeout(0.7));
    NamedCommands.registerCommand("Stop Shooter Wheels", Commands.run(m_shooter::stopMotor, m_shooter));
    NamedCommands.registerCommand("Run Index", Commands.run(m_Intake::runIntake).withTimeout(0.25));
    NamedCommands.registerCommand("Run Intake with Sensor", Commands.run(m_Intake::runIntakeWithSensor));
    NamedCommands.registerCommand("Arm to Intake", Commands.run(() -> {
      m_arm.setMotor(ArmConstants.kArmOffsetRads);
      },
      m_arm).withTimeout(.8));
    NamedCommands.registerCommand("Arm to Podium", Commands.run(() -> {
      m_arm.setMotor(ArmConstants.kPodiumSpot);
      },
      m_arm).withTimeout(1));
      NamedCommands.registerCommand("Arm to Mid", Commands.run(() -> {
        m_arm.setMotor(ArmConstants.kMiddleSpot);
        },
        m_arm).withTimeout(1));
        NamedCommands.registerCommand("Arm to Middle Auto ", Commands.run(() -> {
        m_arm.setMotor(ArmConstants.kMiddleAutoSpot);
        },
        m_arm).withTimeout(1.5));
          NamedCommands.registerCommand("Arm to WingLine", Commands.run(() -> {
        m_arm.setMotor(ArmConstants.kWingLineSpot);
        },
        m_arm).withTimeout(0.7));
         NamedCommands.registerCommand("Arm to Podium Auto", Commands.run(() -> {
        m_arm.setMotor(ArmConstants.kPodiumAutoSpot);
        },
        m_arm).withTimeout(1.5));
      NamedCommands.registerCommand("Stop Intake", Commands.run(m_Intake::stopIntake, m_Intake));
      
    // Build an auto chooser. This will use Commands.none() as the default option.
    // autoChooser = AutoBuilder.buildAutoChooser("W1C1");


    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
      autoChooser = AutoBuilder.buildAutoChooser("W1C1");
    SmartDashboard.putData("Auto Chooser", autoChooser);
    field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
          // Do whatever you want with the pose here
          field.getObject("target pose").setPose(pose);
      });

      // Logging callback for the active path, this is sent as a list of poses
      PathPlannerLogging.setLogActivePathCallback((poses) -> {
          // Do whatever you want with the poses here
          field.getObject("path").setPoses(poses);
      });
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
   
    //Driver Controls
    driverController.R1().onTrue((new InstantCommand(drivebase::zeroGyro)));
     driverController.R2().whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
     driverController.square().onTrue(Commands.run(m_shooter::setMotorTrapSpeed, m_shooter)).onFalse(Commands.run(m_shooter::stopMotor, m_shooter));
    // driverController.L2().whileTrue(new RepeatCommand(new InstantCommand(drivebase::aimAtTarget)));//.onFalse(new TeleopDrive(
    //     drivebase,
     //    () -> MathUtil.applyDeadband(-driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
     //  () -> MathUtil.applyDeadband(-driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
     // () -> -driverController.getRawAxis(2), () -> true));
     // Limelight drivebase targeting
     driverController.L2().onTrue(new TeleopDrive( drivebase,
        () -> MathUtil.applyDeadband(-driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> -drivebase.calculateTurnAngle(), () -> true)).onFalse(new TeleopDrive(
        drivebase,
        () -> MathUtil.applyDeadband(-driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverController.getRawAxis(2), () -> true));
    // drive robot oriented
    driverController.L1().toggleOnTrue(new TeleopDrive(drivebase,
        () -> MathUtil.applyDeadband(-driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverController.getRawAxis(2), () -> false));
    //Climber Pid Control
      //Climbing in middle of chain
    //   driverController
    // .cross()
    // .onTrue(
    //   Commands.run(() -> {
    //   m_climb.setBothMotors(ClimbConstants.kMiddleClimbSpot);
    //   },
    //   m_climb)
    // );
    //  driverController
    // .triangle()
    // .onTrue(
    //   Commands.run(() -> {
    //   m_climb.setBothMotors(ClimbConstants.kEdgeClimbSpot);
    //   },
    //   m_climb)
    // );
     driverController
    .cross()
    .onTrue(
      Commands.run(() -> {
      m_arm.setMotor(ArmConstants.kWingLineSpot);
      },
      m_arm)
    );
    
   // driverController.R2().onTrue(new InstantCommand(drivebase::addFakeVisionReading));
   //Operator Controls
   
   m_operatorController.L2().onTrue(new ParallelCommandGroup(Commands.run(m_Intake::runOuttake, m_Intake), Commands.run(m_shooter::setMotorReverse, m_shooter)))
   .onFalse(new ParallelCommandGroup(Commands.run(m_Intake::stopIntake, m_Intake), Commands.run(m_shooter::stopMotor, m_shooter)));
    m_operatorController.R1().onTrue(Commands.run(m_shooter::setMotorFullSpeed, m_shooter)).onFalse(Commands.run(m_shooter::stopMotor, m_shooter));
    m_operatorController.R2().onTrue(Commands.run(m_Intake::runIntake, m_Intake)).onFalse(Commands.run(m_Intake::stopIntake, m_Intake));
     m_operatorController.povUp().onTrue(Commands.run(m_climb::setRightMotorUp, m_climb)).onFalse(Commands.run(m_climb::stopRightMotor, m_climb));
     m_operatorController.povDown().onTrue(Commands.run(m_climb::setRightMotorDown, m_climb)).onFalse(Commands.run(m_climb::stopRightMotor, m_climb));
     m_operatorController.button(13).onTrue(Commands.run(m_climb::setLeftMotorUp, m_climb)).onFalse(Commands.run(m_climb::stopLeftMotor, m_climb));
     m_operatorController.button(15).onTrue(Commands.run(m_climb::setLeftMotorDown, m_climb)).onFalse(Commands.run(m_climb::stopLeftMotor, m_climb));
     m_operatorController.povLeft().onTrue(Commands.run(m_climb::setBothMotorsUp, m_climb)).onFalse(Commands.run(m_climb::stopBothMotors, m_climb));
      m_operatorController.povRight().onTrue(Commands.run(m_climb::setBothMotorsDown, m_climb)).onFalse(Commands.run(m_climb::stopBothMotors, m_climb));
    // m_operatorController.button(14).onTrue(Commands.run(m_LED::setLEDColorRed, m_LED));
     
  
     //Intake setpoint and run LEDs: if just one sensor is detected, display yellow, if both are detected, display green   
    m_operatorController
    .cross()
    .onTrue(
      new ParallelCommandGroup(
        Commands.run(
            () -> {
              m_arm.setMotor(ArmConstants.kArmOffsetRads);
            },
            m_arm), Commands.run(m_Intake::runIntakeWithSensor, m_Intake),
            Commands.run(m_LED::setLEDColorYellow))); 
  // Stow Setpoint
    m_operatorController
    .circle()
    .onTrue(
      Commands.run(() -> {
      m_arm.setMotor(ArmConstants.kStowSpot);
      },
      m_arm));
    // Subwoofer Setpoint
     m_operatorController
    .triangle()
    .onTrue(
      Commands.run(() -> {
      m_arm.setMotor(ArmConstants.kSubwooferSpot);
      },
      m_arm)
    );
    // Podium Setpoint
     m_operatorController
    .square()
    .onTrue(
      Commands.run(() -> {
      m_arm.setMotor(ArmConstants.kPodiumSpot);
      },
      m_arm)
    );
    //  Middle Line Setpoint
      m_operatorController
    .options()
    .onTrue(
      Commands.run(() -> {
      m_arm.setMotor(ArmConstants.kMiddleSpot);
      },
      m_arm)
    );
    //  Hoard Setpoint
      m_operatorController
    .button(9)
    .onTrue(
      Commands.run(() -> {
      m_arm.setMotor(ArmConstants.kHoardSpot);
      },
      m_arm)
    );
    // Amp Setpoint with shooter revved up
     m_operatorController
    .L1()
    .onTrue(
      new ParallelCommandGroup(Commands.run(() -> {
      m_arm.setMotor(ArmConstants.kAmpSpot);
      }, m_arm),Commands.run(m_shooter::setMotorHalfSpeed, m_shooter)
      
    )).onFalse(Commands.run(m_shooter::stopMotor, m_shooter));
       
    // m_operatorController
    // .options()
    // .onTrue(
    //   Commands.run(() -> {
    //   m_arm.setMotor(m_arm.calculateArmAngle());
    //   },
    //   m_arm)
    // );
   
    // Stop the arm
    m_operatorController.button(12)
    .onTrue(
      Commands.run(()->{
        m_arm.stopMotor();
      }
        , m_arm)
    );


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
   //return drivebase.getAutonomousCommand("W1 to C1 slow", true);
   return autoChooser.getSelected();
  // return new SequentialCommandGroup( new ParallelCommandGroup(Commands.run(() -> {
  //     m_arm.setMotor(ArmConstants.kSubwooferSpot);
  //     },
  //     m_arm).withTimeout(2), Commands.run(m_shooter::setMotorFullSpeed, m_shooter).withTimeout(2)), 
  //     new ParallelCommandGroup(Commands.run(m_Intake::runIntake, m_Intake).withTimeout(2),
  //     Commands.run(m_shooter::stopMotor, m_shooter),
  //     drivebase.getAutonomousCommand("Sub to W4", true),
  //     drivebase.getAutonomousCommand("W4 to Sub", true)));
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
