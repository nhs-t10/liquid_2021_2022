package org.firstinspires.ftc.teamcode.__compiledautoauto;

import static org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.AutoautoProgram.P;
import static org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.Location.L;
import static org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.State.A;
import static org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.Statepath.S;
import static org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.statements.AfterStatement.W;
import static org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.statements.FunctionCallStatement.F;
import static org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.statements.GotoStatement.G;
import static org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.statements.NextStatement.N;
import static org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.values.AutoautoNumericValue.C;
import static org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.values.AutoautoString.U;
import static org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.values.AutoautoUnitValue.E;
import static org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.values.FunctionCall.M;

import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.AutoautoProgram;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.AutoautoProgramElement;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.Location;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.State;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.Statepath;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.statements.AfterStatement;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.statements.FunctionCallStatement;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.statements.GotoStatement;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.statements.NextStatement;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.statements.Statement;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.values.AutoautoNumericValue;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.values.AutoautoString;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.values.AutoautoUnitValue;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.values.AutoautoValue;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.values.FunctionCall;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.AutoautoMacro;

import java.util.HashMap;

//import org.firstinspires.ftc.teamcode.auxilary.buildhistory.BuildHistory;

@SuppressWarnings("WrongPackageStatement")
public class TestMacro__macro_autoauto extends AutoautoMacro {
    public static AutoautoProgram program = generateProgram();

    public static void sL(AutoautoProgramElement e, Location l) {
        e.setLocation(l);
    }

    /*USER_DEFINED_FUNCTIONS*/

    public static AutoautoProgram generateProgram() {
final String h="setServoPosition",i="i",l="arm",u="s",p0="end";

AutoautoUnitValue t=E(3,u);AutoautoNumericValue n=C(3),j0=C(1);AutoautoString k=U(l),h0=U(l);AutoautoValue[]j={k,n},g0={h0,
j0};FunctionCall g=M(h,j),d0=M(h,g0);NextStatement x=N();FunctionCallStatement f=F(g),c0=F(d0);GotoStatement o0=G(p0);AfterStatement s=W(t,
x);Statement[]$e=new Statement[]{f,s},$b0={c0,o0};State e=A($e),b0=A($b0);State[]$d=new State[]{e,b0};Statepath d=S($d,i);HashMap<String,
Statepath> c = new HashMap<String,Statepath>();c.put(i,d);AutoautoProgram w0 = P(c,i);sL(k,L(i,0,2,18));sL(n,L(i,0,2,25));sL(g,
L(i,0,2,1));sL(f,L(i,0,2,1));sL(t,L(i,0,2,35));sL(x,L(i,0,2,38));sL(s,L(i,0,2,29));sL(e,L(i,0,2,1));sL(h0,L(i,1,3,18));sL(j0,
L(i,1,3,25));sL(d0,L(i,1,3,1));sL(c0,L(i,1,3,1));sL(o0,L(i,1,3,29));sL(b0,L(i,1,2,43));return w0;
    }
}
