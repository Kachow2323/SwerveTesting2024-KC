package frc.robot.subsystems;


import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import frc.robot.States;
import frc.robot.States.ArmPos;
import frc.robot.States.HookPos;

public class LED extends SubsystemBase { 
    private static Spark spark;
    public static ArmPos armPosition;
    public static HookPos hookPosition;

    private static ColorSensorV3 colorSensor;
    private static LED instance;
    private static ColorMatch colorMatcher = new ColorMatch();

    private final Color kBlackTarget = new Color(0.0, 0.0, 0.0);
    
    

    public LED (){
        armPosition = ArmPos.STOW;
        hookPosition = HookPos.STOW;
        spark = new Spark(1);
        colorSensor = new ColorSensorV3(Port.kOnboard);
        colorMatcher.addColorMatch(kBlackTarget);
    }
    public void lightStateCheck(ArmPos pos) {
        switch (pos) {
            case STOW:
                spark.set(0.99);
                break;
            case CLIMB_DOWN:
                spark.set(-0.53);
                break;
            case CLIMB_UP:
                spark.set(0.83);
                break;
            case SCORE:
                spark.set(0.53);
                break;
            default:
                spark.set(-0.53);
                break;
        }
    }
   
    
    @Override
    public void periodic() {
        String colorString;
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
        SmartDashboard.putString("COLOR", colorSensor.getColor().toString());
        
        if(match.color == kBlackTarget){
            lightStateCheck(ArmPos.SCORE);
            System.out.println("Setting To Black");
        }else{
            lightStateCheck(ArmPos.STOW);
        }
        // if (colorSensor.getColor().equals(Color.kBlack)) {
        //     spark.set(-0.57);
        // } else spark.set(0.99);
        spark.set(0.99);
        SmartDashboard.putNumber("Current LED", spark.get());
        //else armAndHookStateCheck(armPosition, hookPosition);
    }

    public static LED getInstance() {
        if(instance == null) instance = new LED();
        return instance;

    }
}



