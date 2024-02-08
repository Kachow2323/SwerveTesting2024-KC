package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.States;
import frc.robot.States.ArmPos;
import frc.robot.States.HookPos;

public class LED extends SubsystemBase { 
    Spark spark;
    public static ArmPos armPosition;
    public static HookPos hookPosition;
    public LED (){
        spark = new Spark(0);
    }
    public void armAndHookStateCheck(ArmPos armPosition, HookPos hookPosition) {
        switch (armPosition) {
            case STOW:
                spark.set(-0.75);
                break;
            case SCORE:
                spark.set(-0.5);
                break;
        }
        switch(hookPosition){
            case OPEN:
                spark.set(-0.25);
            default:
                break;
            
        }
    }
   
    
    @Override
    public void periodic() {
        armAndHookStateCheck(armPosition, hookPosition);
        super.periodic();
    }
}




