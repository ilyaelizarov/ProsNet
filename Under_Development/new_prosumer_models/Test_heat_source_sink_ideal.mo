within ProsNet.Under_Development.new_prosumer_models;
model Test_heat_source_sink_ideal
  heat_source_sink heat_source_sink_ideal1
    annotation (Placement(transformation(extent={{-24,2},{32,48}})));
  Modelica.Fluid.Sources.MassFlowSource_T boundary(
    use_m_flow_in=true,
    use_T_in=true,
    redeclare package Medium =
        Media.Water,
    nPorts=1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-20,-70})));
  Modelica.Fluid.Vessels.OpenTank tank(
    height=5,
    crossArea=10,
    use_portsData=false,
    redeclare package Medium =
        Media.Water,
    nPorts=1) annotation (Placement(transformation(extent={{30,-76},{70,-34}})));
  Modelica.Blocks.Sources.RealExpression temperature_sec(y=273.15 + 70)
    "Kelvin"
    annotation (Placement(transformation(extent={{-68,68},{-48,88}})));
  Modelica.Blocks.Sources.RealExpression temperature_in(y=273.15 + 30)
    "Kelvin"
    annotation (Placement(transformation(extent={{-96,-88},{-76,-68}})));
  Modelica.Blocks.Sources.RealExpression m_flow(y=5/60) "kg/s"
    annotation (Placement(transformation(extent={{-96,-70},{-76,-50}})));
  inner Modelica.Fluid.System system
    annotation (Placement(transformation(extent={{56,62},{76,82}})));
equation
  connect(boundary.m_flow_in, m_flow.y) annotation (Line(points={{-28,-80},{
          -44,-80},{-44,-60},{-75,-60}}, color={0,0,127}));
  connect(boundary.T_in, temperature_in.y) annotation (Line(points={{-24,-82},
          {-24,-88},{-70,-88},{-70,-78},{-75,-78}}, color={0,0,127}));
  connect(temperature_sec.y, heat_source_sink_ideal1.T_set)
    annotation (Line(points={{-47,78},{4,78},{4,48}}, color={0,0,127}));
  connect(heat_source_sink_ideal1.port_cold, boundary.ports[1]) annotation (
      Line(points={{20.8,2},{20.8,-54},{-20,-54},{-20,-60}}, color={0,127,255}));
  connect(heat_source_sink_ideal1.port_hot, tank.ports[1]) annotation (Line(
        points={{-12.8,2},{-12.8,-56},{24,-56},{24,-84},{50,-84},{50,-76}},
        color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(lineColor = {75,138,73},
                fillColor={255,255,255},
                fillPattern = FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(lineColor = {0,0,255},
                fillColor = {75,138,73},
                pattern = LinePattern.None,
                fillPattern = FillPattern.Solid,
                points={{-36,60},{64,0},{-36,-60},{-36,60}})}),  Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Test_heat_source_sink_ideal;
