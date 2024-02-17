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

public static Command getPathPlannerCommandAmp() {
    return new PathPlannerAuto("Simple Auto Part 1");
  }

  public static Command getPathPlannerCommandExitStartingLine(){
    return new PathPlannerAuto("Simple Auto Part 2");
  }

  /**
   * Drives to AMP. Scores 1 NOTE. Leave Starting Line and drives to far side of the *
   * Starting Pos, closest to AMP, hugging the subwoofer
   */

  public static Command ScoreAutoOneNoteAmp(){
    return new SequentialCommandGroup(
        Auto.getPathPlannerCommandAmp(),
        new RunCommand(() -> RobotContainer.getInstance().arm.setArmState(States.ArmPos.SCORE), RobotContainer.getInstance().arm),
        new WaitCommand(1.0),
        new RunCommand(() -> RobotContainer.getInstance().hook.setHookState(States.HookPos.OPEN), RobotContainer.getInstance().hook),
        new WaitCommand(0.5),
        new RunCommand(() -> RobotContainer.getInstance().arm.setArmState(States.ArmPos.STOW), RobotContainer.getInstance().arm),
        new RunCommand(() -> RobotContainer.getInstance().hook.setHookState(States.HookPos.STOW), RobotContainer.getInstance().hook),
        Auto.getPathPlannerCommandExitStartingLine()
        );
  }

}