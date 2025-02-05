package org.firstinspires.ftc.teamcode.eventHandlers;
import org.firstinspires.ftc.teamcode.enums.LiftStatus;
import org.firstinspires.ftc.teamcode.eventListeners.LiftPositionListener;
import java.util.List;
import java.util.ArrayList;


public class LiftEventHandler {
    private final List<LiftPositionListener> listeners = new ArrayList<>();

    // Add a listener
    public void addListener(LiftPositionListener listener) {
        listeners.add(listener);
    }

    // Notify all listeners about the event
    public void notifyListeners(LiftStatus position) {
        for (LiftPositionListener listener : listeners) {
            listener.onPositionReached(position);
        }
    }
}
