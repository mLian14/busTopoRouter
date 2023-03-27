package parser;

import shapes.BaseType;
import shapes.Obstacle;
import shapes.PseudoBase;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;

public class DocumentParser {
    private Document parseDoc;

    public DocumentParser() {
        this.parseDoc = new Document();
    }

    public void processInput(String[] input){
        /*
        Read files
         */
        for (int i = 0; i < input.length; ++i){
            if (input[i].equals("")) continue;
            else if (input[i].contains("slaveC")){
                this.parseDoc.setSlaveC(Integer.parseInt(input[i].substring(7)));
            }
            else if (input[i].contains("busC")){
                this.parseDoc.setBusC(Integer.parseInt(input[i].substring(5)));
            }
            else if (input[i].equals("Uni_Keepouts")){
                i++;
                int uni_o_cnt = 0;
                while (!input[i].equals("FIN")){
                    uni_o_cnt++;
                    String[] coordinates = input[i].split(" ");
                    Obstacle o = new Obstacle(("o" + uni_o_cnt), Integer.parseInt(coordinates[0]), Integer.parseInt(coordinates[1]), Integer.parseInt(coordinates[2]), Integer.parseInt(coordinates[3]));
                    parseDoc.addToUni_keepouts(o);
                    i++;
                }
            }
            else if (input[i].equals("Master")){
                i++;
                String[] coordinates = input[i].split(" ");
                PseudoBase master = new PseudoBase(Integer.parseInt(coordinates[1]), Integer.parseInt(coordinates[2]));
                master.setName(coordinates[0]);
                master.setType(BaseType.Master);
                parseDoc.setMaster(master);
            }
            else if (input[i].equals("Slave")){
                i++;
                while (!input[i].equals("FIN")){
                    String[] coordinates = input[i].split(" ");
                    PseudoBase sv = new PseudoBase(Integer.parseInt(coordinates[1]), Integer.parseInt(coordinates[2]));
                    sv.setName(coordinates[0]);
                    sv.setType(BaseType.Slave);
                    parseDoc.addToSlaves(sv);
                    i++;
                }
            }



        }

    }

    private String[] readFiles(String path) {
        File dir = new File(path);
        File[] files = dir.listFiles();
        ArrayList<String> allLines = new ArrayList<>();
        ArrayList<String> lines;
        for (File file : files) {
            try (BufferedReader reader = new BufferedReader(new InputStreamReader(new FileInputStream(file), StandardCharsets.UTF_8))) {
                lines = new ArrayList<>();
                String line;
                while ((line = reader.readLine()) != null) {
                    lines.add(line);
                }
                allLines.addAll(lines);

            } catch (IOException e) {
                e.printStackTrace();
            }

        }
        return allLines.toArray(new String[]{});
    }

    public void parseInputToDocument(String path) {
        this.processInput(this.readFiles(path));
    }

    public Document getParseDoc() {
        return parseDoc;
    }
}
