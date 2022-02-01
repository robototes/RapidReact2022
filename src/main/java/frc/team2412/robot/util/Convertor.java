package frc.team2412.robot.util;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVRecord;

import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class Convertor {
    public static void fileToJSON(String path){
        File csvData = new File("/path/to/csv");
        CSVParser parser = CSVParser.parse(csvData, CSVFormat.RFC4180);
        for (CSVRecord csvRecord : parser) {

        }
    }

    public static void csvToJson(CSV csvFile){
        try (InputStream in = new FileInputStream(csvFile);) {
            CSV csv = new CSV(true, ',', in );
            List< String > fieldNames = null;
            if (csv.hasNext()) fieldNames = new ArrayList< >(csv.next());
            List <Map< String, String >> list = new ArrayList < > ();
            while (csv.hasNext()) {
                List < String > x = csv.next();
                Map < String, String > obj = new LinkedHashMap< >();
                for (int i = 0; i < fieldNames.size(); i++) {
                    obj.put(fieldNames.get(i), x.get(i));
                }
                list.add(obj);
            }
            ObjectMapper mapper = new ObjectMapper();
            mapper.enable(SerializationFeature.INDENT_OUTPUT);
            mapper.writeValue(System.out, list);
        }
    }
}
