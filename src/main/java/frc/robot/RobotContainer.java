// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HookConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Eyes;
import frc.robot.subsystems.Hook;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

/*
 * This class is where the bulk of the robot (including the subsystems) should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // 
  /*READ ME:
  The robot's subsystems
  */
  public final DriveSubsystem m_robotDrive;
  public final Arm arm;
  public final Hook hook;
  public final LED led;
  
  public final Field2d field;

  /*READ ME:
  Creates 2 Xbox Controllers for the Operator and Driver
  Operator is under Port 1 which is set under Operator Constants in Constants.java
  Driver is under Port 0 which is set under Driver Constants in Constants.java
  The class XBoxController is Native to WPILIB and require no external vendordeps
  */

  private static RobotContainer instance = null;
  private static final XboxController operatorController = new XboxController(Constants.OIConstants.operatorController);
  private static final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);

/*READ ME:
  Sets up the driver and operator buttons on each controller by intializing the buttons (pre-known placements)
  By pre-mapping each button's location, we can choose which buttons are used and gives us the whole range of the buttons on each XBox Controllers
  Most of the Buttons will never be used but thats ok. Notice the 9+ unused variables. Leave them alone.
  */

  public static final Trigger driver_A = new JoystickButton(driverController, 1),
    driver_B = new JoystickButton(driverController, 2), driver_X = new JoystickButton(driverController, 3),
    driver_Y = new JoystickButton(driverController, 4), driver_LB = new JoystickButton(driverController, 5),
    driver_RB = new JoystickButton(driverController, 6), driver_VIEW = new JoystickButton(driverController, 7),
    driver_MENU = new JoystickButton(driverController, 8);
  private static final Trigger operator_A = new JoystickButton(operatorController, 1),
    operator_B = new JoystickButton(operatorController, 2), operator_X = new JoystickButton(operatorController, 3),
    operator_Y = new JoystickButton(operatorController, 4), operator_LB = new JoystickButton(operatorController, 5),
    operator_RB = new JoystickButton(operatorController, 6), operator_VIEW = new JoystickButton(operatorController, 7),
    operator_MENU = new JoystickButton(operatorController, 8);
  
  private static final POVButton operator_DPAD_UP = new POVButton(operatorController, 0),
    operator_DPAD_RIGHT = new POVButton(operatorController, 90), operator_DPAD_DOWN = new POVButton(operatorController, 180),
    operator_DPAD_LEFT = new POVButton(operatorController, 270);
  private static final POVButton driver_DPAD_UP = new POVButton(driverController, 0),
    driver_DPAD_RIGHT = new POVButton(driverController, 90), driver_DPAD_DOWN = new POVButton(driverController, 180),
    driver_DPAD_LEFT = new POVButton(driverController, 270);

   /*READ ME:
  A static instance of the Robot Container with all its contents
  */

  public static RobotContainer getInstance() {
      if(instance == null) instance = new RobotContainer();
      return instance;
  }
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    arm = Arm.getInstance();
    hook = Hook.getInstance();
    m_robotDrive = new DriveSubsystem();
    led = LED.getInstance();

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

     /*READ ME:
  The default drive command which is defined in DriveSubsystems.java is the main method of drive used in our 2024 Drivetrain.
  The .drive method takes in 5 parameters which is defined in DriveSubsystems.java (5 parameters: 3 doubles, 2 booleans.)  
  The 3 doubles are the Left Joysticks X & Y values (-1 thru 1) and Right Joysticks X values (-1 thru 1). - XBoxController
  Applies a simple deadband onto the read values before passing them into the the method to minimize stick drift
  If we want field relative control, we set field relative to true and if we want to limit the jerkyness of the drive, we can set rate limit to true
  Rate limit is basically setting the limit of one request. (ie: controller requests 1.00 but we limit it to 0.8)
  */

    field = new Field2d();
    
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      field.setRobotPose(pose);
    });
  }

  public Command goToPositionBezier(Pose2d target) {
    Rotation2d bezierControlPointHeading = Rotation2d.fromRadians(
      Math.atan2(
        target.getY() - m_robotDrive.getPose().getY(),
        target.getX() - m_robotDrive.getPose().getX()
      )
    );
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(
          m_robotDrive.getPose().getX(),
          m_robotDrive.getPose().getY(),  
          bezierControlPointHeading
        ),
        new Pose2d(target.getX(), target.getY(), target.getRotation())
    );

    return AutoBuilder.followPath(
      new PathPlannerPath(
        bezierPoints, 
        Constants.AutoConstants.pathConstraints,
        new GoalEndState(0.0, target.getRotation())
      )
    );
  }

  /**
   * Create and return a command to follow a path that takes the robot to the target pose.
   *
   * @return The current alliance of the robot.
   */
  public Command goToPosition(Pose2d target, float goalEndVelocity, float rotationDelayDistance) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = Constants.AutoConstants.pathConstraints;
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
            target,
            constraints,
            goalEndVelocity, // Goal end velocity in meters/sec
            rotationDelayDistance // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
    return pathfindingCommand;
  }

  public Command pathFindtoPath(String pathname) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = Constants.AutoConstants.pathConstraints;
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile(pathname),
            constraints,
            2.
           );
    return pathfindingCommand;
  }

  public Command goToPositionFromPath(String pathname) {
    // Load the path we want to pathfind to and follow
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathname);

    // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
    PathConstraints constraints = new PathConstraints(
            3.0, 3.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
            path,
            constraints,
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );

    return pathfindingCommand;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */

   /*READ ME:
  Using the above defined button bindings for each of the XboxControllers, we setting commands for them to follow
  Many of the bindings rely/use the different subsystems (ie: arm, hook, auto)
  Should be pretty self-explanatory (runCommands on logic conditionals)
  */

  private void configureButtonBindings() {
    driver_DPAD_RIGHT.whileTrue(
      new RunCommand(() -> arm.setOpenLoop(.2), arm))
        .onFalse(new InstantCommand(() -> arm.setOpenLoop(0))
        );

    driver_DPAD_LEFT.whileTrue(
      new RunCommand(() -> arm.setOpenLoop(-.2), arm))
        .onFalse(new InstantCommand(() -> arm.setOpenLoop(0))
        );

    driver_DPAD_UP.whileTrue(
      new RunCommand(() -> hook.setOpenLoop(0.1), hook))
        .onFalse(new InstantCommand(() -> hook.setOpenLoop(0))
        );

    driver_DPAD_DOWN
    .whileTrue(
      new RunCommand(() -> hook.setOpenLoop(-0.1), hook))
        .onFalse(new InstantCommand(() -> hook.setOpenLoop(0))
        );

    driver_RB
      .whileTrue(
        new RunCommand(() -> hook.setHookState(States.HookPos.OPEN), hook)
      );
    driver_LB
      .whileTrue(
        new RunCommand(() -> hook.setHookState(States.HookPos.STOW), hook)
      );

    // driver_X.whileTrue(
    //   goToPosition(
    //     new Pose2d(0.85, 1.18, Rotation2d.fromDegrees(-90)),
    //     0, 
    //     0)
    // );

    // driver_Y.whileTrue(
    //   pathFindtoPath("FlyTest")
    // );

    driver_Y.whileTrue(
      goToPosition(
        new Pose2d(0.85, 1.18, Rotation2d.fromDegrees(90)), 0, 0)
    );

    driver_X.whileTrue(
      goToPositionBezier(
        new Pose2d(0.85, 1.18, Rotation2d.fromDegrees(90))
      )
    );

    // driver_B.whileTrue(
    //   goToPositionBezier(
    //     new Pose2d(0, 0, Rotation2d.fromDegrees(0))
    //   )

    // );

   /* READ ME:
     * This command runs the SCORE command for the AMP shot in every attempt execpt AUTO
     * By condensing the entire score command into one method we no longer have to keep defining it everywhere and we set the standard for each attempt
     * Utilizes Constants.java for realtive and absoulte scoring encoder values.
     * Parrallel Command Group - The command runs at the same time but we put a time delay to calculate the exact timing
     * We needed the wait command bc we need the momentum from the swinign arm to score into the AMP
     */

    operator_Y
      .whileTrue(
        new ParallelCommandGroup(
          new RunCommand(() -> {
            arm.setArmState(States.ArmPos.SCORE);
            }, arm),
          new SequentialCommandGroup(
            new WaitCommand(HookConstants.delay),
            new RunCommand(() -> {
              hook.setHookState(States.HookPos.SCORE);
            }, hook
            )
          )
        )
      );

    /* READ ME:
     * This command runs the STOW command for the AMP shot in every attempt execpt AUTO
     * Utilizes Constants.java for realtive and absoulte scoring encoder values.
     * Parrallel Command Group - The command runs at the same time but we put a time delay to calculate the exact timing
     * Exact Replica of the SCORE basically without time delay
     */

    operator_X
      .whileTrue(
       new RunCommand(() -> {
        arm.setArmState(States.ArmPos.STOW); 
        hook.setHookState(States.HookPos.STOW);
       }, arm, hook)
      );

    /* READ ME:
     * This command runs the Auto-Align command for the AMP shot - WIP
     * Utilizes Eyes.java and Robot.java to run vision code and calculate the wanted rotation rate
     * Aligns with the amp along the Z-axis rotationally but will not align with the amp from the X&Y-axis transversely
     */

    operator_B
      .whileTrue(
       new RunCommand(() -> {
        SmartDashboard.putNumber("B button Working", 1);
         double rotate = Robot.getAprilTagOffset();
         m_robotDrive.drive(0,0, rotate, true, false);

       }, m_robotDrive)
      );

      /* READ ME:
     * This command runs the CLIMBUP & CLIMBDOWN command for the endgame chain climb in TELEOP
     * Utilizes Constants.java for realtive and absoulte scoring encoder values.
     * Exact Replica of the SCORE basically without time delay
     */

    operator_RB
      .whileTrue(
       new RunCommand(() -> {
        arm.setArmState(States.ArmPos.CLIMB_UP); 
       }, arm)
      );
    operator_LB
      .whileTrue(
       new RunCommand(() -> {
        arm.setArmState(States.ArmPos.CLIMB_DOWN); 
       }, arm)
      );

      
  }

  // public Command stowArm() {
  //   return new RunCommand(() -> arm.setArmState(States.ArmPos.STOW), arm);
  // }

  // public Command scoreArm(){
  //   return new RunCommand(() -> arm.setArmState(States.ArmPos.SCORE), arm);
  // }

  // public Command stowHook() {
  //   return new RunCommand(() -> hook.setHookState(States.HookPos.STOW), hook);
  // }

  // public Command score() {
  //   return new RunCommand(() -> {
  //       arm.setArmState(States.ArmPos.SCORE); 
  //       hook.setHookState(States.HookPos.SCORE);
  //      }, arm, hook);
  // }

  /* READ ME:
     * This command runs the SCORE command for the AMP shot in AUTO
     * By condensing the entire score command into one method we no longer have to keep defining it everywhere and we set the standard for each attempt
     * Utilizes Constants.java for realtive and absoulte scoring encoder values.
     * Parrallel Command Group - The command runs at the same time but we put a time delay to calculate the exact timing
     * We needed the wait command bc we need the momentum from the swinign arm to score into the AMP
     */

  public Command scoreHookDelay() {
    return new ParallelCommandGroup(
          new RunCommand(() -> {
            arm.setArmState(States.ArmPos.SCORE);
            }, arm),
          new SequentialCommandGroup(
            new WaitCommand(HookConstants.delay),
            new RunCommand(() -> {
              hook.setHookState(States.HookPos.SCORE);
            }, hook
            )
          )
        );
  }

  // public void bindOI(){

  //   // driver_X
  //   //     .onTrue(stowArm());
    
  //   driver_Y
  //       .onTrue(scoreArm());

 
  // }

  /**
   * Returns the current alliance, with false indicating blue and true indicating red.
   * If there is no alliance, blue alliance is assumed. 
   *
   * @return The current alliance of the robot.
   */
  public boolean getAlliance() {
     var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getPathPlannerCommand1() {
  //   return new PathPlannerAuto("Simple Auto Part 1");
  // }

  // public Command getPathPlannerCommand2(){
  //   return new PathPlannerAuto("Simple Auto Part 2");
  // }

  //Returns Bool? from FMS
  public static Optional<Alliance> isRedAlliance(){
    return DriverStation.getAlliance();
  }

}

/* 2023-2024 For-TEA-Simo Java Code by:

Kaden J. Chow - Programming Lead - MHS 2026 - https://github.com/Kachow2323
Ronit Barman - Tech Captain - MHS 2024
Joshua Seo - Programming - MHS 2027
Adam Situ - Asst. Programming Lead - MHS 2027
Michelle Y - Asst. Programming Lead - MHS 2025

With Invaluable Help from:
  Mentor John :D
  Mentor Lauren :D
  Mentor Katie :D
  Everyone else from inside and outside of 253
*/