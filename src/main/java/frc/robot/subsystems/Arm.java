package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
    private CANSparkMax Motor = new CANSparkMax(9, MotorType.kBrushless);

    public Arm() {

    }

    public void move(double power) {
        Motor.set(power);
    }
}
