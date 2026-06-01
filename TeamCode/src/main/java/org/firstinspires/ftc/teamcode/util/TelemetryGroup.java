package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryGroup {
    private Telemetry telemetry;
    private TelemetryManager telemetryM;

    public TelemetryGroup(Telemetry telemetry, TelemetryManager telemetryM) {
        this.telemetry = telemetry;
        this.telemetryM = telemetryM;
    }

    public void addData(String caption, Object value) {
        telemetry.addData(caption, value);
        telemetryM.addData(caption, value);
    }

    public void update() {
        telemetry.update();
        telemetryM.update();
    }
}
