package frc.robot.subsystems.climber;
/* 
*   The use of SparkMax motors are is a placeholder.
*   We can swap this out for the actual motor controller we use for the climber later.
*/

public class Climber {

    private final ClimberIO climberIO;
    private final ClimberIO.ClimberIOInputs inputs = new ClimberIO.ClimberIOInputs();

    public Climber(ClimberIO climberio) {
        this.climberIO = climberio;
    }

    public void setMotorOpenLoop(double percent) {
        this.climberIO.setMotorOpenLoop(percent);
    }

    public void setMotorVelocity(double velocityRps) {
        this.climberIO.setMotorVelocity(velocityRps);
    }

    public void stopMotor() {
        this.climberIO.stopMotor();
    }


}