package parser;

import shapes.Obstacle;
import shapes.PseudoBase;

import java.util.ArrayList;

public class Document {
    private String name;
    private PseudoBase master;
    private ArrayList<PseudoBase> slaves;
    private ArrayList<Obstacle> obstacles;
    private int busC;
    private int slaveC;

    public Document() {
        this.slaves = new ArrayList<>();
        this.obstacles = new ArrayList<>();

    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public PseudoBase getMaster() {
        return master;
    }

    public void setMaster(PseudoBase master) {
        this.master = master;
    }

    public ArrayList<PseudoBase> getSlaves() {
        return slaves;
    }

    public void addToSlaves(PseudoBase slave) {
        this.slaves.add(slave);
    }

    public ArrayList<Obstacle> getObstacles() {
        return obstacles;
    }

    public void addToUni_keepouts(Obstacle o) {
        this.obstacles.add(o);
    }

    public int getBusC() {
        return busC;
    }

    public void setBusC(int busC) {
        this.busC = busC;
    }

    public int getSlaveC() {
        return slaveC;
    }

    public void setSlaveC(int slaveC) {
        this.slaveC = slaveC;
    }


}
