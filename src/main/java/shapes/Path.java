package shapes;

import java.util.ArrayList;

/**
 * @auther lianmeng
 * @create 29.03.23
 */
public class Path {
    private ArrayList<PseudoBase> nodes;

    public Path() {
        this.nodes = new ArrayList<>();
    }

    public ArrayList<PseudoBase> getNodes() {
        return nodes;
    }

    public void addToNodes(PseudoBase node) {
        this.nodes.add(node);
    }
}
