within ProsNet.Fluid.FixedResistances.Examples;
model ProsPresDrop
   extends Modelica.Icons.Example;

  replaceable package Medium = ProsNet.Media.Water;

  parameter Modelica.Units.SI.Length diameter=0.022;

  Modelica.Fluid.Fittings.SimpleGenericOrifice resLoc(
    redeclare package Medium = Medium,
    diameter=diameter,
    zeta=3.5) annotation (Placement(transformation(extent={{38,18},{24,6}})));
  Modelica.Fluid.Pipes.DynamicPipe pipe(
    redeclare package Medium = Medium,
    diameter=diameter,
    length=5,
    roughness=0.011e-3) annotation (Placement(visible=true, transformation(
        origin={-24,12},
        extent={{-10,10},{10,-10}},
        rotation=0)));
  inner Modelica.Fluid.System system
    annotation (Placement(transformation(extent={{60,60},{80,80}})));
  Sources.MassFlowSource_T boundary(m_flow=0.358, nPorts=1)
    annotation (Placement(transformation(extent={{-84,4},{-64,24}})));
  Sources.Boundary_pT bou(nPorts=1)
    annotation (Placement(transformation(extent={{92,0},{72,22}})));
  Sensors.RelativePressure senRelPre
    annotation (Placement(transformation(extent={{-10,-38},{10,-18}})));
  Modelica.Blocks.Interaction.Show.RealValue realValue(significantDigits=3)
    annotation (Placement(transformation(extent={{46,-70},{80,-50}})));
equation
  connect(pipe.port_b, resLoc.port_b)
    annotation (Line(points={{-14,12},{24,12}}, color={0,127,255}));
  connect(resLoc.port_a, bou.ports[1]) annotation (Line(points={{38,12},{56,12},
          {56,11},{72,11}}, color={0,127,255}));
  connect(pipe.port_a, boundary.ports[1]) annotation (Line(points={{-34,12},{-50,
          12},{-50,14},{-64,14}}, color={0,127,255}));
  connect(senRelPre.port_b, resLoc.port_a) annotation (Line(points={{10,-28},{32,
          -28},{32,-26},{38,-26},{38,12}}, color={0,127,255}));
  connect(senRelPre.port_a, pipe.port_a) annotation (Line(points={{-10,-28},{-34,
          -28},{-34,12}}, color={0,127,255}));
  connect(senRelPre.p_rel, realValue.numberPort)
    annotation (Line(points={{0,-37},{0,-60},{43.45,-60}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ProsPresDrop;
