/*
 * Copyright (c) 2021. Meng Lian and Yushen Zhang
 *
 *   CONFIDENTIAL
 *   __________________
 *
 *   Meng Lian and Yushen Zhang
 *   All Rights Reserved.
 *
 *   NOTICE:  All information contained herein is, and remains
 *   the property of Meng Lian, Yushen Zhang and the Technical University
 *   of Munich, if applies. The intellectual and technical concepts contained
 *   herein are proprietary to Meng Lian, Yushen Zhang and/or the Technical University
 *   of Munich and may be covered by European and Foreign Patents,
 *   patents in process, and are protected by trade secret or copyright law.
 *   Dissemination of this information or reproduction of this material
 *   is strictly forbidden unless prior written permission is obtained
 *   from Meng Lian and Yushen Zhang.
 */

package grb;

public class GurobiVariables {
    private GurobiVariable var;
    private GurobiVariable var2;

    public GurobiVariables(GurobiVariable var, GurobiVariable var2) {
        this.var = var;
        this.var2 = var2;
    }

    public GurobiVariables(GurobiVariable var) {
        this.var = var;
        this.var2 = null;
    }

    public GurobiVariable getVar() {
        return var;
    }

    public void setVar(GurobiVariable var) {
        this.var = var;
    }

    public GurobiVariable getVar2() {
        return var2;
    }

    public void setVar2(GurobiVariable var2) {
        this.var2 = var2;
    }

    public boolean has2Vars(){
        return this.var2!=null;
    }

    public boolean hasVar(GurobiVariable v){
        return (this.var.equals(v) || this.var2.equals(v));
    }

    @Override
    public String toString() {
        return "GurobiVariables{" +
                "var=" + var +
                ", var2=" + var2 +
                '}';
    }
}
