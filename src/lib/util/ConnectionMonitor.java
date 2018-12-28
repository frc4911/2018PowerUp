package lib.util;

public class ConnectionMonitor {

    public static double kConnectionTimeoutSec = 1.0;
    private double mLastPacketTime;
    private LatchedBoolean mJustReconnected;
    private LatchedBoolean mJustDisconnected;

    public ConnectionMonitor() {
        mLastPacketTime = 0.0;
        mJustReconnected = new LatchedBoolean();
        mJustDisconnected = new LatchedBoolean();
    }

    public void onLoop(double timestamp) {
        synchronized (ConnectionMonitor.this) {
            boolean has_connection = true;
            if (timestamp - mLastPacketTime > kConnectionTimeoutSec) {
                has_connection = false;
            }

            if (mJustReconnected.update(has_connection)) {
                // Reconfigure blink if we are just connected.
            	CrashTracker.logConnectionEnabled();
            }

            if (mJustDisconnected.update(!has_connection)) {
                // Reconfigure blink if we are just disconnected.
            	CrashTracker.logConnectionDisabled();
            }
        }
    }
}
