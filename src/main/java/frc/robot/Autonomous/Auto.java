package frc.robot.Autonomous;

import frc.robot.subsystems.*;
import frc.robot.*;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Auto {

  /* READ ME:
   * Our Autos consist of 3 parts:
   * Go to Score Pos, Score Note Action, Back Out across Alliance Zone
   * No Ambiguity between RED or BLUE Alliance Auto - Flips Paths
   * Forces itself into the amp by running into for a set period of time - prevent bounce back
   */

public static Command getPathPlannerCommandAmp() {
    return new PathPlannerAuto("Simple Auto Part 1");
  }

  public static Command getPathPlannerCommandExitStartingLine(){
    return new PathPlannerAuto("Simple Auto Part 2");
  }

  /* READ ME:
   * Play Off Auto
   * No Ambiguity between RED or BLUE Alliance Auto - Flips Paths
   * Forces itself into the amp by running into for a set period of time - prevent bounce back
   */

  public static Command getPathPlannerCommandFarAmp(){
    return new PathPlannerAuto("PlayoffAuto");
  } 

  public static Command getPathPlannerCommandAutoLeave(){
    return new PathPlannerAuto("PlayoffAuto2");
  } 


  /* READ ME:
   * Play Off Auto
   * Starts at Amp far side, waits for 2743 to finish their auto (time delay), then goes to score 1 amp.
   * No Ambiguity between RED or BLUE Alliance Auto - Flips Paths
   * Error: Was offset due to non-comp treads were being used (~3.5% error @ 1m)
   */

   public static Command ScorePlayoffAuto(){
    return new SequentialCommandGroup(
      new WaitCommand(8.5),
      Auto.getPathPlannerCommandFarAmp(),
      new WaitCommand(.2),
      RobotContainer.getInstance().scoreHookDelay().withTimeout(1.5),
      new WaitCommand(.1),
      new InstantCommand(() -> RobotContainer.getInstance().arm.setArmState(States.ArmPos.STOW), RobotContainer.getInstance().arm),
      new InstantCommand(() -> RobotContainer.getInstance().hook.setHookState(States.HookPos.STOW), RobotContainer.getInstance().hook),
      new WaitCommand(0),
      Auto.getPathPlannerCommandAutoLeave()
    );
  }

  /* READ ME:
   * Drives to AMP. Scores 1 NOTE. Leave Starting Line and drives to far side of the *
   * Starting Pos, closest to AMP, hugging the subwoofer
   * No Ambiguity between RED or BLUE Alliance Auto - Flips Paths
   */

  public static Command ScoreAutoOneNoteAmp(){
    return new SequentialCommandGroup(
      Auto.getPathPlannerCommandAmp(),
      new WaitCommand(1.0),
      RobotContainer.getInstance().scoreHookDelay().withTimeout(2.),
      new WaitCommand(1.),
      new InstantCommand(() -> RobotContainer.getInstance().arm.setArmState(States.ArmPos.STOW), RobotContainer.getInstance().arm),
      new InstantCommand(() -> RobotContainer.getInstance().hook.setHookState(States.HookPos.STOW), RobotContainer.getInstance().hook),
      new WaitCommand(1.0),
      Auto.getPathPlannerCommandExitStartingLine()
    );
  }

  /* READ ME:
   * Drives on a time based system. (FC)
   * Inputs: 4 Doubles, 2 Booleans
   */

  public static Command driveTime (double xspeed, double ySpeed, double rot, double sec){
    return new RunCommand(
      () -> RobotContainer.getInstance().m_robotDrive.drive(
          xspeed,
          ySpeed,
          rot,
          true,
          true),
      RobotContainer.getInstance().m_robotDrive
      ).withTimeout(sec);
  }

  public static Command driveAutoCommand(){
    return new PathPlannerAuto("B_DriveAwayStraight3mAuto");
    // INSERT AUTO NAME INTO THE CHOICE! 
  }

  // =========================
  // Legacy Code - MBR 2024

// public static Command RedAmp(){
  //   return new PathPlannerAuto("Simple Auto Part 1 Red");
  // }

  // public static Command RedExit(){
  //   return new PathPlannerAuto("Simple Auto Part 2 Red");
  // }

// public static Command RedAmpAuto(){
//     return new SequentialCommandGroup(
//       Auto.RedAmp(),
//       new WaitCommand(1.),
//       RobotContainer.getInstance().scoreHookDelay().withTimeout(2.),
//       new WaitCommand(1.),
//       new InstantCommand(() -> RobotContainer.getInstance().arm.setArmState(States.ArmPos.STOW), RobotContainer.getInstance().arm),
//       new InstantCommand(() -> RobotContainer.getInstance().hook.setHookState(States.HookPos.STOW), RobotContainer.getInstance().hook),
//       new WaitCommand(1),
//       Auto.RedExit()
//     );
//   }

  }
