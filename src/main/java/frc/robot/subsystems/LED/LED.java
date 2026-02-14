package frc.robot.subsystems.LED;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{

    private final LEDIO ledIO;
    private final LEDIO.LEDIOInputs inputs = new LEDIO.LEDIOInputs();
    

    public LED(LEDIO ledio) {
        this.ledIO = ledio;
    }

    public void updateInputs(LEDIO.LEDIOInputs inputs) {
        this.ledIO.updateInputs(inputs);
    }
    
    public void sendMessage1() {
        System.out.println("Message1 sent!");
        this.ledIO.sendMessage1();
    }

    @Override
    public void periodic() {
        this.ledIO.updateInputs(this.inputs);
        Logger.recordOutput("/LED/LED Connected", this.inputs.LEDConnected);
        Logger.recordOutput("/LED/Last LED Message", this.inputs.lastLEDMessage);
    }
    
    // public void update() {
    //     this.ledIO.updateInputs(this.inputs);
    //     Logger.recordOutput("LED Connected", this.inputs.LEDConnected);
    //     Logger.recordOutput("Last LED Message", this.inputs.lastLEDMessage);
    // }
}
