package frc.robot.subsystems;
/* 
*   The use of SparkMax motors are is a placeholder.
*   We can swap this out for the actual motor controller we use for the climber later.
*/
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.MotorType;

public class Climber {
    private final CANSparkMax motor = new CANSparkMax(10, MotorType.kBrushless);

    public void setClimberSpeed(double speed) {
        motor.set(speed);
    }
}