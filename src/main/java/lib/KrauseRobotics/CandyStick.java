package lib.KrauseRobotics;
import edu.wpi.first.hal.can.CANJNI;;

public class CandyStick implements AutoCloseable {
    private int m_CANId;

    public CandyStick(int id) {
        m_CANId = id;
    }

	@Override
	public void close() throws Exception {

	}
    
}
