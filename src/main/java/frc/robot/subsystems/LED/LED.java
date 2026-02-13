package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LED {

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

    public void update() {
        this.ledIO.updateInputs(this.inputs);
        SmartDashboard.putBoolean("LED Connected", this.inputs.LEDConnected);
        SmartDashboard.putString("Last LED Message", this.inputs.lastLEDMessage);
    }
}
