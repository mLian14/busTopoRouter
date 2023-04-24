import gurobi.GRBException;
import parser.OutputDocument;
import processor.Processor;

import java.io.IOException;
import java.time.Duration;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;

/**
 * @auther lianmeng
 * @create 26.03.23
 */
public class test {

    public static void main(String[] args) throws IOException, GRBException {
        LocalDateTime start = LocalDateTime.now();
        System.out.println("Program Starts at: " + DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss.SSS").format(start));
        Processor processor = new Processor();

        OutputDocument output = processor.processToOutputPre("input_busTopoRouter/case1_I2C4_SCL");
        //OutputDocument output = processor.processToOutputPre("input_busTopoRouter/test");
        LocalDateTime end = LocalDateTime.now();
        Duration duration = Duration.between(start, end);
        LocalDateTime duration_formated = LocalDateTime.ofInstant(java.time.Instant.ofEpochMilli(duration.toMillis()), ZoneId.of("UTC"));
        System.out.println("Program Ends at: " + DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss.SSS").format(end));
        System.out.println("Program Run Time is: " + DateTimeFormatter.ofPattern("HH:mm:ss.SSS").format(duration_formated));

    }

}
