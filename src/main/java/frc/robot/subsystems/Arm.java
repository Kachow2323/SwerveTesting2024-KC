
package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.States;
import frc.robot.Constants.ArmConstants;
import frc.utils.Util;


public class Arm extends SubsystemBase {
    /* READ ME:
     * Creates 2 SparkMAX objects with their CAN ID and Motor Type
     * We know we have to make one motor follow another motor to sync them up (master & slave motors)
     * We use encoder values from the internal CANcoder in the SparkMAX to measure revolutions
     * Use PID Feedback Control Loops to reach armPosition in revolutions which we stored in Constants.java
     * All Arm Movements are Relative to the armReset Pos at the Hard Stop
     */
    private static final CANSparkMax motorR = Util.createSparkMAX(ArmConstants.rightArmMotorID, MotorType.kBrushless);
    private static final CANSparkMax motorL = Util.createSparkMAX(ArmConstants.leftArmMotorID, MotorType.kBrushless);
    private RelativeEncoder relArmEncoder = motorR.getEncoder();

    /* READ ME:
    * Creates a PID Controller which we use to control the motors movement
    */
    private SparkPIDController pidController;
    
    private static Arm instance;
    public static Arm getInstance() {
        if(instance == null) instance = new Arm();
        return instance;

    }
    
    private Arm() {
        resetEncoders();
        motorR.setSmartCurrentLimit(18);
        motorL.setSmartCurrentLimit(18);
        motorR.setInverted(false);
        motorL.follow(motorR, true); // Slave motors to leading motor to sync them together
        motorR.setIdleMode(IdleMode.kBrake);
        motorL.setIdleMode(IdleMode.kBrake);
        pidController = motorR.getPIDController(); //Sets the control of the right motor to speed that the PID controller commanded
        pidController.setP(ArmConstants.kP); //Proportional Gain 
        pidController.setI(ArmConstants.kI);// Intergral Gain
        pidController.setD(ArmConstants.kD); // Derivative Gain
        pidController.setIZone(0); // DW will be 0 for most of the time
        pidController.setOutputRange(ArmConstants.pidOutputLow, ArmConstants.pidOutputHigh); // Max and Min output so the robot does rip itself apart
        register();
    }



    public void setOpenLoop(double value) {
        SmartDashboard.putNumber("Arm Commanded arm actuation", value);
        motorR.set(value);
        motorL.set(value);
    }
    
    public void stopArm() {
        setOpenLoop(0);
    }
    
    /**
     * Resets encoders to zero
     */
    public void resetEncoders() {
        relArmEncoder.setPosition(0.0);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("right Arm Relative encoder value", relArmEncoder.getPosition());
        SmartDashboard.putNumber("Right Arm current", motorR.getOutputCurrent());
    }
    
    /* READ ME:
     * Select the Setpoint aka reference point for the PID Controller
     */
    public void setArmPosition(double position) {
        pidController.setReference(position, ControlType.kPosition);
        SmartDashboard.putNumber("Arm SetPoint", position);
    }

    /* READ ME:
     * Depending on what we input in (ie: button matching), we can select which setpoint we want to go to.
     * Also used to set the stateCheck for the LED's
     */
    public void setArmState(States.ArmPos state) {
        SmartDashboard.putNumber("Position", state.val);
        switch (state) {
            case STOW:
                pidController.setP(ArmConstants.kP);
                setArmPosition(ArmConstants.stow);
                LED.armPosition = States.ArmPos.STOW;
                break;
            case CLIMB_UP:
                pidController.setP(ArmConstants.climbP);
                setArmPosition(ArmConstants.climb_up);
                LED.armPosition = States.ArmPos.CLIMB_UP;
                break;
            case CLIMB_DOWN:
                pidController.setP(ArmConstants.climbP);
                setArmPosition(ArmConstants.stow);
                LED.armPosition = States.ArmPos.CLIMB_DOWN;
                break;
            default:
                pidController.setP(ArmConstants.kP);
                setArmPosition(ArmConstants.score);
                LED.armPosition = States.ArmPos.SCORE;
                break;
        }
    }

    public void setArmPositionDegree(double degreePosition) { //define degreePosition earlier
        double convertDeg = 11.375;
        double encoderPosition = degreePosition*convertDeg; //degree to encoder
        double currentPosition = relArmEncoder.getPosition(); 
        SmartDashboard.putNumber("degreePosition", degreePosition);
        SmartDashboard.putNumber("encoder value", encoderPosition);
        SmartDashboard.putNumber("current arm position", currentPosition);
    }
  
}
