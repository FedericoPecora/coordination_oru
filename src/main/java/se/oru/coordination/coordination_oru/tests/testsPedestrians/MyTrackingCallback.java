package se.oru.coordination.coordination_oru.tests.testsPedestrians;

import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulationWithPedestrians;
import se.oru.coordination.coordination_oru.util.ColorPrint;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class MyTrackingCallback {

    static int criticalSections = 0;
    static String pathFileName;

    public static synchronized void writeToLogFile(String token, String fileName) {
        File file = new File("journal_logs/" + fileName);
        FileWriter fileWriter = null;
        try {
            fileWriter = new FileWriter(file, true);
            fileWriter.write(token);
        } catch (IOException e) {
            ColorPrint.error("Couldn't write to file.");
            e.printStackTrace();
        } finally {
            try {
                fileWriter.close();
            } catch (IOException e) {
                ColorPrint.error("Couldn't close file.");
                e.printStackTrace();
            }
        }

    }
}