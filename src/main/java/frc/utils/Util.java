package frc.utils;


import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.Constants;

public class Util {

    private static int talonSRXDefaultContinuousLimit = 38;
    private static int talonSRXDefaultPeakLimit = 45;
    private static int talonSRXDefaultPeakDuration = 125;

    public static final boolean talonFXStatorLimitEnable = false;
    public static final double talonFXStatorCurrentLimit = 100;
    public static final double talonFXStatorTriggerThreshold = 100;
    public static final double talonFXStatorTriggerDuration = 0;

    public static final boolean talonFXSupplyLimitEnable = false;
    public static final double talonFXSupplyCurrentLimit = 70;
    public static final double talonFXSupplyTriggerThreshold = 70;
    public static final double talonFXSupplyTriggerDuration = 0.7;

    private static int sparkMAXDefaultCurrentLimit = 40;

    private static double voltageCompensation = Constants.kMaxVoltage;
    

    /**
     * Create a CANSparkMax with current limiting enabled
     * 
     * @param id the ID of the Spark MAX
     * @param motortype the type of motor the Spark MAX is connected to 
     * @param stallLimit the current limit to set at stall
     * 
     * @return a fully configured CANSparkMAX
     */
    public static CANSparkMax createSparkMAX(int id, MotorType motortype, int stallLimit) {
        CANSparkMax sparkMAX = new CANSparkMax(id, motortype);
        /*sparkMAX.restoreFactoryDefaults();
        sparkMAX.enableVoltageCompensation(voltageCompensation);
        sparkMAX.setSmartCurrentLimit(stallLimit);
        sparkMAX.setIdleMode(IdleMode.kCoast);*/

        //sparkMAX.burnFlash();
        return sparkMAX;
    }

    /**
     * Create a CANSparkMax with default current limiting enabled
     * 
     * @param id the ID of the Spark MAX
     * @param motortype the type of motor the Spark MAX is connected to
     * 
     * @return a fully configured CANSparkMAX
     */
    public static CANSparkMax createSparkMAX(int id, MotorType motortype) {
        return createSparkMAX(id, motortype, sparkMAXDefaultCurrentLimit);
    }
}