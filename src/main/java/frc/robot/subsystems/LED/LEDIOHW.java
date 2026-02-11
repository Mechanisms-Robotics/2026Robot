package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.CAN;

public class LEDIOHW implements LEDIO {
    CAN can = new CAN(1);

    public void updateInputs(LEDIOInputs inputs) {
        // Update any inputs you need to log here
    }
    
    public void sendMessage1() {
        can.writePacket(new byte[12], 0);
    }
}