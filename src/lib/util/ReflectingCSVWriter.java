package lib.util;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.concurrent.ConcurrentLinkedDeque;

/**
 * Writes data to a CSV file
 * 
 * Borrowed from team 254
 */
public class ReflectingCSVWriter<T> {
    ConcurrentLinkedDeque<String> mLinesToWrite = new ConcurrentLinkedDeque<>();
    PrintWriter mOutput = null;
    Field[] mFields;

    public ReflectingCSVWriter(String fileName, Class<T> typeClass) {
        mFields = typeClass.getFields();
        try {
        	// Keep last non-zero byte file around
			try {
	        	Path srcPath = Paths.get(fileName);
	        	if (Files.exists(srcPath) && Files.size(srcPath) > 0) {
					Path dstPath = Paths.get(fileName.replace(".csv", ".last.csv"));
					Files.move(srcPath, dstPath, StandardCopyOption.REPLACE_EXISTING);
				}
		    } catch (IOException e) {
		        e.printStackTrace();
		    }
            mOutput = new PrintWriter(fileName);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        
        // Write field names.
        StringBuffer line = new StringBuffer();
        for (Field field : mFields) {
            if (line.length() != 0) {
                line.append(", ");
            }
            line.append(field.getName());
        }
        writeLine(line.toString());
    }

    public void add(T value) {
        StringBuffer line = new StringBuffer();
        for (Field field : mFields) {
            if (line.length() != 0) {
                line.append(", ");
            }
            try {
                line.append(field.get(value).toString());
            } catch (IllegalArgumentException e) {
                e.printStackTrace();
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
        mLinesToWrite.add(line.toString());
    }

    protected synchronized void writeLine(String line) {
        if (mOutput != null) {
            mOutput.println(line);
        }
    }

    // Call this periodically from any thread to write to disk.
    public void write() {
        while (true) {
            String val = mLinesToWrite.pollFirst();
            if (val == null) {
                break;
            }
            writeLine(val);
        }
    }

    public synchronized void flush() {
        if (mOutput != null) {
            write();
            mOutput.flush();
        }
    }
}