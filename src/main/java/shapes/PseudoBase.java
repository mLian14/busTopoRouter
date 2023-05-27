package shapes;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

/**
 * @auther lianmeng
 * @create 25.03.23
 */
public class PseudoBase {

    private int x, y;
    private String name;
    private BaseType type;

    /*
    0: oqUL
    1: oqUR
    2: oqLR
    3: oqLL
    4: oqL
    5: oqR
    6: oqT
    7: oqB
     */
    private Map<Obstacle, int[]> pseudo_oDir_qs;

    /*
    opposite relation: 45-degree
    0: oqtL
    1: oqtR
    2: oqbL
    3: oqbR
    opposite relation: rectangular
    4: oqdL
    5: oqdR
    6: oqdT
    7: oqdB
     */
    private Map<Obstacle, int[]> pseudo_oRel_qs;


    /*

     */
    private Map<Obstacle, int[]> pseudo_iVars;




    public PseudoBase(int x, int y) {
        this.x = x;
        this.y = y;
        this.pseudo_oDir_qs = new HashMap<>();
        this.pseudo_oRel_qs = new HashMap<>();
        this.pseudo_iVars = new HashMap<>();




    }

    public BaseType getType() {
        return type;
    }

    public void setType(BaseType type) {
        this.type = type;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public int getX() {
        return x;
    }

    public void setX(int x) {
        this.x = x;
    }

    public int getY() {
        return y;
    }

    public void setY(int y) {
        this.y = y;
    }

    public Map<Obstacle, int[]> getPseudo_oDir_qs() {
        return pseudo_oDir_qs;
    }

    public void addToPseudo_oDir_qs(Obstacle o, int[] q) {
        this.pseudo_oDir_qs.put(o, q);
    }

    public Map<Obstacle, int[]> getPseudo_oRel_qs() {
        return pseudo_oRel_qs;
    }

    public void addToPseudo_oRel_qs(Obstacle o, int[] q) {
        this.pseudo_oRel_qs.put(o, q);
    }


    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        PseudoBase point = (PseudoBase) o;
        return Double.compare(point.x, x) == 0 && Double.compare(point.y, y) == 0;
    }

    public String convertMapIntArrayToString(Map<Obstacle, int[]> mapIntArray) {
        StringBuilder mapAsString = new StringBuilder("||");
        for (Obstacle o : mapIntArray.keySet()) {
            mapAsString.append(o.getName() + "[" + Arrays.toString(mapIntArray.get(o)) + "];");
        }
        mapAsString.append("||");
        return mapAsString.toString();
    }


    @Override
    public String toString() {
        ArrayList<Integer> tmpQ = new ArrayList<>();


        return "PseudoBase{" +
                "x=" + x +
                ", y=" + y +
                ", name='" + name +
                ", mapQ = " + convertMapIntArrayToString(pseudo_oDir_qs) +
                ", mapIq = " + convertMapIntArrayToString(pseudo_iVars) +
                '\'' +
                '}';
    }


}

