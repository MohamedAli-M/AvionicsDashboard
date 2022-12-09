import Instruments.ADI;
import Instruments.HSI;
import com.fazecast.jSerialComm.*;

import java.nio.charset.StandardCharsets;
import javax.swing.*;

@SuppressWarnings("serial")
public class Main extends JFrame
{

    public static void main(String[] args) throws Exception
    {
		SerialPort serial = null;
		try{
			serial = SerialPort.getCommPorts()[4];
			serial.setBaudRate(115200);
			serial.openPort();
			serial.setComPortTimeouts(SerialPort.TIMEOUT_READ_BLOCKING, 1000, 0);
			serial.getInputStream();
		} catch(Exception e){
			System.out.println("Serial port is invalid. Try again.");
			System.exit(1);
		}


        HSI hsi = new HSI();
        ADI adi = new ADI();

		float roll = 0;
		float pitch = 0;
		float yaw = 0;

		System.out.println("Initializing the IMU ..");

		while(hsi.isShowing() || adi.isShowing()){
			try {
				while (serial.bytesAvailable() == 0)
					Thread.sleep(20);

				byte[] readBuffer = new byte[serial.bytesAvailable()];
				serial.readBytes(readBuffer, readBuffer.length);
				String s = new String(readBuffer, StandardCharsets.UTF_8);

				String[] values = s.split(" ", 3);

				yaw = Float.parseFloat(values[0]);
				pitch = Float.parseFloat(values[1]);
				roll = Float.parseFloat(values[2]);

			} catch (Exception e) {
				;
			}

			//adi
			adi.setPitchBankValues(Math.toRadians(roll), Math.toRadians(pitch));

			//hsi
			hsi.setHeading(Math.toRadians(yaw));

			//repaint
        	hsi.repaint();
        	adi.repaint();
        };
		serial.closePort();
    }
}