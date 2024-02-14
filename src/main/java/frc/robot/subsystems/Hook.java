package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.HookConstants;
import frc.robot.States;
import frc.utils.Util;

public class Hook extends SubsystemBase {
    private static final CANSparkMax motor = Util.createSparkMAX(HookConstants.motorID, MotorType.kBrushless, 20);
    private SparkAbsoluteEncoder hookEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    private RelativeEncoder relHookEncoder = motor.getEncoder();

    private SparkPIDController pidController;
    public double enable = 0;
    
    
    private static Hook instance;
    public static Hook getInstance() {
        if(instance == null) instance = new Hook();
        return instance;

    }
    
    
    private Hook() {
    
    
        resetEncoders();
       
        motor.setInverted(false);
       
        motor.setIdleMode(IdleMode.kBrake);

        pidController = motor.getPIDController();
        pidController.setFeedbackDevice(hookEncoder); //Currently set to rel encoder
        pidController.setP(HookConstants.kP); //0.1
        pidController.setI(HookConstants.kI);//0.01
        pidController.setD(HookConstants.kD);
        pidController.setIZone(0);
        pidController.setFF(0);
        pidController.setOutputRange(HookConstants.pidOutputLow, HookConstants.pidOutputHigh);
        register();
    }



    public void setOpenLoop(double value) {
        SmartDashboard.putNumber("Arm Commanded arm actuation", value);
        motor.set(value);
        
    }
    
    public void stopArm() {
    
        setOpenLoop(0);
    }
    
    /**
     * Resets encoders to zero
     */
    public void resetEncoders() {
        relHookEncoder.setPosition(0.0);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("hook abs encoder", hookEncoder.getPosition());
        SmartDashboard.putNumber("hook rel encoder", relHookEncoder.getPosition());
        SmartDashboard.putNumber("Hook Voltage", motor.getBusVoltage());
        SmartDashboard.putNumber("Hook Output Current", motor.getOutputCurrent());
        SmartDashboard.putNumber("Score set", enable);
    }
    


    public void setHookPosition(double position) {
        pidController.setReference(position, ControlType.kPosition);
        SmartDashboard.putNumber("Hook SetPoint", position);
        enable = position;
    }

    public void setHookPositionDegree(double degreePosition) { //define degreePosition earlier
        double convertDeg = 11.375;
        double encoderPosition = degreePosition*convertDeg; //degree to encoder
        double currentPosition = hookEncoder.getPosition(); 
        SmartDashboard.putNumber("degreePosition", degreePosition);
        SmartDashboard.putNumber("encoder value", encoderPosition);
        SmartDashboard.putNumber("current arm position", currentPosition);
        
    }

    public void setHookState(States.HookPos state) {
        switch (state) {
            case STOW:
                setHookPosition(HookConstants.stow);
                break;
            case OPEN:
                setHookPosition(HookConstants.open);
                break;
            default:
                setHookPosition(HookConstants.score);
                break;
        }
    }
}
