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
import edu.wpi.first.wpilibj.I2C;
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

    // IR Sensor from REV Color Sensore V3
    private final static I2C.Port i2cPort = I2C.Port.kOnboard;
    private final static ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private  static double IR = m_colorSensor.getIR();

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
                spark.set(0.57);
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
        double IR = m_colorSensor.getIR();
        SmartDashboard.putNumber("IR", IR);
        int proximity = m_colorSensor.getProximity();
        SmartDashboard.putNumber("Proximity", proximity);

        if(proximity >= 161){
            System.out.println("Detected");
            lightStateCheck(ArmPos.SCORE);
        }else{
            System.out.print("Not detected");
            lightStateCheck(armPosition);
        }
        // if (colorSensor.getColor().equals(Color.kBlack)) {
        //     spark.set(-0.57);
        // } else spark.set(0.99);
        //spark.set(0.99);
        SmartDashboard.putNumber("Current LED", spark.get());
        //else armAndHookStateCheck(armPosition, hookPosition);
        
    }

    public static LED getInstance() {
        if(instance == null) instance = new LED();
        return instance;

    }
}




