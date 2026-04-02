package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.SerialPort;
 
public class LEDController {
    private SerialPort serial;
 
    public LEDController() {
        try {
            serial = new SerialPort(9600, SerialPort.Port.kUSB);
            serial.setWriteBufferMode(SerialPort.WriteBufferMode.kFlushOnAccess);
        } catch (Exception e) {
            System.err.println("[LEDController] Failed to open serial port: " + e.getMessage());
            serial = null;
        }
    }
 
    // Simple state only
    public void set(LEDState state) {
        set(state, 0, false);
    }
 
    // State + RPM% + april tag
    public void set(LEDState state, int rpmPercent, boolean hasAprilTag) {
        if (serial == null) return;
        try {
            int clampedRPM = Math.min(100, Math.max(0, rpmPercent));
            // 0-100 = no april tag, 101-201 = has april tag
            int encodedData = hasAprilTag ? clampedRPM + 101 : clampedRPM;
            byte[] data = {
                (byte) state.id,
                (byte) encodedData
            };
            serial.write(data, 2);
        } catch (Exception e) {
            System.err.println("[LEDController] Serial write failed: " + e.getMessage());
        }
    }
 
    public void close() {
        if (serial != null) serial.close();
    }
}