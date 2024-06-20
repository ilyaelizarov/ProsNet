within ProsNet.Controls;
block TankChargingController
  "Controller to enable or disable storage tank charging"

  Modelica.Blocks.Sources.RealExpression realExpression2(y=338)
    annotation (Placement(transformation(extent={{-120,18},{-94,42}})));
  Controls.Building_Controls.OBC.CDL.Reals.Subtract sub annotation (
      Placement(transformation(extent={{-52,-10},{-32,10}})));
  Controls.Building_Controls.OBC.CDL.Reals.Hysteresis cha(uLow=-5,
      uHigh=0)
    annotation (Placement(transformation(extent={{-8,-10},{12,10}})));
  Controls.Building_Controls.OBC.CDL.Reals.Subtract sub1 annotation (
      Placement(transformation(extent={{-54,-70},{-34,-50}})));

  Controls.Building_Controls.OBC.CDL.Logical.Latch lat
    annotation (Placement(transformation(extent={{46,-8},{66,12}})));
  Controls.Building_Controls.OBC.CDL.Logical.Not not1 annotation (
      Placement(transformation(extent={{26,-70},{46,-50}})));
  Controls.Building_Controls.OBC.CDL.Interfaces.RealInput TTanTop
    annotation (Placement(transformation(extent={{-158,-26},{-118,14}})));
  Controls.Building_Controls.OBC.CDL.Interfaces.RealInput TTanTopSet
    annotation (Placement(transformation(extent={{-156,-74},{-116,-34}})));
  Controls.Building_Controls.OBC.CDL.Interfaces.RealInput TTanBot
    annotation (Placement(transformation(extent={{-156,-124},{-116,
            -84}})));
  Controls.Building_Controls.OBC.CDL.Interfaces.BooleanOutput charge
    annotation (Placement(transformation(extent={{94,-18},{134,22}})));
  Controls.Building_Controls.OBC.CDL.Reals.Hysteresis cha1(uLow=-5,
      uHigh=0) annotation (Placement(transformation(extent={{-16,-70},
            {4,-50}})));
equation
  connect(realExpression2.y, sub.u1) annotation (Line(points={{-92.7,
          30},{-62,30},{-62,6},{-54,6}}, color={0,0,127}));
  connect(sub.y, cha.u)
    annotation (Line(points={{-30,0},{-10,0}}, color={0,0,127}));
  connect(cha.y, lat.u) annotation (Line(points={{14,0},{36,0},{36,2},
          {44,2}}, color={255,0,255}));
  connect(not1.y, lat.clr) annotation (Line(points={{48,-60},{56,-60},
          {56,-12},{44,-12},{44,-4}}, color={255,0,255}));
  connect(TTanBot, sub1.u2) annotation (Line(points={{-136,-104},{-72,
          -104},{-72,-66},{-56,-66}}, color={0,0,127}));
  connect(TTanTopSet, sub1.u1) annotation (Line(points={{-136,-54},{
          -56,-54}}, color={0,0,127}));
  connect(TTanTop, sub.u2)
    annotation (Line(points={{-138,-6},{-54,-6}}, color={0,0,127}));
  connect(lat.y, charge)
    annotation (Line(points={{68,2},{114,2}}, color={255,0,255}));
  connect(sub1.y, cha1.u)
    annotation (Line(points={{-32,-60},{-18,-60}}, color={0,0,127}));
  connect(cha1.y, not1.u)
    annotation (Line(points={{6,-60},{24,-60}}, color={255,0,255}));
  annotation (
  defaultComponentName="tanCha",
  Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,-100},{100,100}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Text(
          textColor={0,0,255},
          extent={{-150,110},{150,150}},
          textString="%name"),
        Text(
          extent={{-98,98},{-48,62}},
          textColor={0,0,127},
          textString="TTanTopSet"),
        Text(
          extent={{-96,50},{-46,14}},
          textColor={0,0,127},
          textString="TTanTop"),
        Text(
          extent={{42,20},{92,-16}},
          textColor={0,0,127},
          textString="charge"),
        Text(
          extent={{-96,-62},{-46,-98}},
          textColor={0,0,127},
          textString="TTanBot"),
        Text(
          extent={{-98,-30},{-48,-66}},
          textColor={0,0,127},
          textString="TTanTopSet")}),
     Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(revisions="<html>
<ul>
<li>
November 15, 2023, by David Blum:<br/>
Add that charging is stopped when bottom temperature reaches set point.
</li>
<li>
October 4, 2023, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>", info="<html>
<p>
Controller that outputs <code>charge=true</code> if the domestic hot
water tank needs to be charged, and <code>false</code> otherwise.
</p>
<p>
Whether the tank needs to be charged depends on the difference between the
measured top and bottom tank temperatures and the tank's set point.
If the top temperature is below the set point with hysteresis,
charging is enabled until the bottom temperature reaches the set point with
hysteresis, at which point charging is disabled.
The hysteresis is <i>5</i> Kelvin.
</p>
</html>"));
end TankChargingController;
