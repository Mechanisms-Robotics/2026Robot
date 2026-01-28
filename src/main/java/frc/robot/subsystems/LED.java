package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;

public class LED {
    CAN can = new CAN(1);

    public void sendMessage1() {
        System.out.println("Message1 attempted");
        can.writePacket(new byte[12], 0);
    }
}