package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.CAN;

public class LEDIOHW implements LEDIO {
    LEDIOInputs inputs = new LEDIOInputs();
    CAN can = new CAN(1);

    private double currentBrightness = 0;
    public void updateInputs(LEDIOInputs inputs) {
        // Update any inputs you need to log here
        this.inputs.LEDConnected = true;
        this.inputs.LEDBrightness = this.currentBrightness;
    }
    
    public void sendMessage1() {
        can.writePacket(new byte[12], 0);
        this.inputs.lastLEDMessage = "Message1 sent!";
    }

    // Placeholder for testing
    public void sendMessage2() {
        can.writePacket(new byte[13], 0);
        this.inputs.lastLEDMessage = "Message2 sent!";
    }
}