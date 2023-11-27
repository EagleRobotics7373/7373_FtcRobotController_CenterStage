package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

public class CameraStreamSplitter {

    private CameraStreamSplitter() {}

    private static final double DEFAULT_FPS = 15.0;

    public static void startCameraStream(CameraStreamSource source, double maxFps) {
        CameraStreamServer.getInstance().setSource(source);
        FtcDashboard.getInstance().startCameraStream(source, maxFps);

    }

    public static void startCameraStream(CameraStreamSource source) {
        startCameraStream(source, DEFAULT_FPS);
    }

    public static void stopCameraStream() {
        CameraStreamServer.getInstance().setSource(null);
        FtcDashboard.getInstance().stopCameraStream();
    }

}
