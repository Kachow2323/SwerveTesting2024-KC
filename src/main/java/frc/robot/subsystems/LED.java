package frc.robot.subsystems;


import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.States;
import frc.robot.States.ArmPos;
import frc.robot.States.HookPos;

public class LED extends SubsystemBase { 
    private static Spark spark;
    public static ArmPos armPosition;
    public static HookPos hookPosition;

    private static ColorSensorV3 colorSensor;
    private static LED instance;

    public LED (){
        armPosition = ArmPos.STOW;
        hookPosition = HookPos.STOW;
        spark = new Spark(1);
        colorSensor = new ColorSensorV3(Port.kOnboard);
    }
    public void armAndHookStateCheck(ArmPos armPosition, HookPos hookPosition) {
        switch (armPosition) {
            case STOW:
                spark.set(-0.53);
                break;
            case CLIMB_DOWN:
                spark.set(-0.53);
                break;
            case CLIMB_UP:
                spark.set(0.53);
                break;
            case SCORE:
                spark.set(-0.53);
                break;
            default:
                spark.set(-0.53);
                break;
        }
    }
   
    
    @Override
    public void periodic() {
        SmartDashboard.putString("COLOR", colorSensor.getColor().toString());
        // if (colorSensor.getColor().equals(Color.kBlack)) {
        //     spark.set(-0.57);
        // } else spark.set(0.99);
        spark.set(-0.17);
        SmartDashboard.putNumber("Current LED", spark.get());
        //else armAndHookStateCheck(armPosition, hookPosition);
    }

    public static LED getInstance() {
        if(instance == null) instance = new LED();
        return instance;

    }
}




