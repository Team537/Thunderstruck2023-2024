package org.firstinspires.ftc.teamcode.Systems.Software;

import android.os.Environment;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.io.Writer;
import java.util.LinkedHashMap;
import java.util.function.Supplier;

public class Burn {

    public static final String directoryName = "FIRST";
    private final String filePath;
    public Supplier dataSupplier;
    public ElapsedTime elapsedTime;
    private Writer fileWriter;

    private LinkedHashMap<Double, String> data = new LinkedHashMap<Double, String>();

    /**
     *
     * @param fileName
     * @param dataSupplier
     * @param elapsedTime
     * @param <T>
     */
    public <T>Burn (String fileName, Supplier<T> dataSupplier, ElapsedTime elapsedTime) {

        // Get the path to the directory where all external data will be stored.
        String directoryPath = Environment.getExternalStorageDirectory().getPath() + "/" + directoryName;

        // Create a file object for the directory.
        File directory = new File(directoryPath);
        directory.mkdir();

        // Set the variable filePath equal to the path of the file.
        filePath = directoryPath + "/" + fileName;

        this.dataSupplier = dataSupplier;
        this.elapsedTime = elapsedTime;

        try {
            fileWriter = new FileWriter(filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    public void log () {
        Object value = dataSupplier.get();
        data.put(elapsedTime.seconds(), String.valueOf(value));
    }

    public void burn () {
        String outputString = "";
        for (LinkedHashMap.Entry<Double,String> entry : data.entrySet()) {
            outputString = outputString + entry.getKey().toString() + ": " + entry.getValue() + "\n";
        }
        try {
            fileWriter.write(outputString);
            fileWriter.flush();
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


}
