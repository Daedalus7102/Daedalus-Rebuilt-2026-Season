package frc.robot.subsystems.led;
import edu.wpi.first.wpilibj.SerialPort;
public class LEDController {
private SerialPort serial;
private String lastCommand = "";
public LEDController() {
try {
serial = new SerialPort(9600, SerialPort.Port.kUSB);
serial.setWriteBufferMode(SerialPort.WriteBufferMode.kFlushOnAccess);
} catch (Exception e) {
System.err.println("[LEDController] Failed to open serial port: " + e.getMessage());
serial = null;
}
}
public void setOff(){ send("OFF");     }  
public void setAuto(){ send("AUTO");    }  

    // Called from FeederSubsystem
public void setIndexing() { send("INTAKE");  } 

    // Called from ShooterSubsystem
public void setShooting() { send("SHOOT");   }  

    // Deduplicated sender — aka no spam
private void send(String cmd) {
if (serial == null) return;
if (cmd.equals(lastCommand)) return;
try {
serial.writeString(cmd + "\n");
lastCommand = cmd;
} catch (Exception e) {
System.err.println("[LEDController] Serial write failed: " + e.getMessage());
}
}
public void close() {
if (serial != null) serial.close();
    }
}