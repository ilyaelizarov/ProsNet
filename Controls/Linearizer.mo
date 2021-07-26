within ProsNet.Controls;
block Linearizer
  "Linearizes flow rate with respect to normalized flow coefficient of a valve"

  replaceable parameter ProsNet.Controls.Data.Linearizer.Generic cha
    constrainedby ProsNet.Controls.Data.Linearizer.Generic
    "Record with the inverse of inherent characteristic" annotation (choicesAllMatching=true,
      Placement(transformation(extent={{58,58},{78,78}})));

  Modelica.Blocks.Interfaces.RealInput kappa
    "Normalized flow coefficient of a valve"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput op "Opening of a valve"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Tables.CombiTable1Dv chaInvValve(final table=cha.inv)
    "Table with inverse of inherent valve characteristic"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));

equation
  connect(kappa, chaInvValve.u[1])
    annotation (Line(points={{-120,0},{-12,0}}, color={0,0,127}));
  connect(chaInvValve.y[1], op)
    annotation (Line(points={{11,0},{110,0}}, color={0,0,127}));
  annotation (defaultComponentName="lin", Icon(graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
      Polygon(lineColor={192,192,192},
          fillColor={192,192,192},
          fillPattern=FillPattern.Solid,
          points={{-78,90},{-86,68},{-70,68},{-78,90}}),
        Line(points={{-78,78},{-78,-90}},
            color={192,192,192}),
      Line(points={{-88,-80},{84,-80}},
          color={192,192,192}),
      Polygon(lineColor={192,192,192},
          fillColor={192,192,192},
          fillPattern=FillPattern.Solid,
          points={{92,-80},{70,-72},{70,-88},{92,-80}}),
        Line(points={{-78,-80},{66,72}}, color={0,0,0}),
          Text(
          extent={{-98,-98},{98,-140}},
          lineColor={28,108,200},
          textString="%name")}),
    Documentation(info="<html>
<p>The linearizer block allows linearizing of the flow rate for the equal-percentage inherent valve characteristic. The normalized flow coefficient serves as an input for the linearizer.</p>
<p>When only a small pressure drop is allowed by a control valve, the most used valve type has an equal percentage inherent characteristic. The volumetric flow rate through such a valve can be expressed as:</p>
<p align=\"center\"><img src=\"modelica://ProsNet/doc/pic/ValveFlowFormula.gif\"/></p>
<p>where <i>K<sub>v</i></sub> is the flow factor [m<sup>3</sup>/(h bar<sup>1/2</sup>)]; <i>f(op)</i> is the inherent valve characteristic, which is a function of the opening [-]; <i>op</i> is the opening of the valve [-]; <i><span style=\"font-family: Symbol;\">D</span>p</i> is a pressure drop at the valve [bar], and SG is the specific gravity of the fluid [-].</p>
<p><br>To linearize the flow rate with respect to the valve characteristic, the opening is substituted with its inverse in this model:</p>
<p align=\"center\"><img src=\"modelica://ProsNet/doc/pic/ValveCha.gif\"/></p>
<p>where <span style=\"font-family: Symbol;\">k</span> is the actual flow coefficient, which varies between zero and one [-].</p>
</html>"));
end Linearizer;
