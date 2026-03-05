package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.SerialPort;

public class LEDController {
	private SerialPort serial;
	private LEDState lastCommand;

	public LEDController() {
		try {
			serial = new SerialPort(9600, SerialPort.Port.kUSB);
			serial.setWriteBufferMode(SerialPort.WriteBufferMode.kFlushOnAccess);
		} catch (Exception e) {
			System.err.println("[LEDController] Failed to open serial port: " + e.getMessage());
			serial = null;
		}
	}

	public void set(LEDState state) {
		if (serial == null) return;
		if (state == lastCommand) return;
		try {
			byte[] data = {(byte) state.id};
			serial.write(data, 1);
			lastCommand = state;
		} catch (Exception e) {
			System.err.println("[LEDController] Serial write failed: " + e.getMessage());
		}
	}

	public void close() {
		if (serial != null) serial.close();
	}
}