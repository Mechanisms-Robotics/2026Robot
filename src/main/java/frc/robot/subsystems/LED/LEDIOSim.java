package frc.robot.subsystems.LED;

public class LEDIOSim implements LEDIO {
    LEDIOInputs inputs = new LEDIOInputs();

    public void updateInputs(LEDIOInputs inputs) {
        // Update any inputs you need to log here
        this.inputs = inputs;
    }
        
    public void sendMessage1() {
        this.inputs.lastLEDMessage = "Message1 sent!";
    }

    // Placeholder for testing
    public void sendMessage2() {
        this.inputs.lastLEDMessage = "Message2 sent!";
    }

}