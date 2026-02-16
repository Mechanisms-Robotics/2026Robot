package frc.robot.subsystems.climber;
/* 
*   The use of SparkMax motors are is a placeholder.
*   We can swap this out for the actual motor controller we use for the climber later.
*/

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{

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

    @Override
    public void periodic() {
        this.climberIO.updateInputs(this.inputs);
        Logger.recordOutput("Climber/Motor Connected", this.inputs.climberConnected);
        Logger.recordOutput("Climber/Motor Velocity RPS", this.inputs.climberVelocityRotationsPerSec);
    }

    // public void update() {
    //     this.climberIO.updateInputs(this.inputs);
    //     Logger.recordOutput("Climber Motor Velocity RPS", this.inputs.climberVelocityRotationsPerSec);
    //     Logger.recordOutput("Climber Motor Connected", this.inputs.climberConnected);
    // }

}