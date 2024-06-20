within ProsNet.Under_Development.new_prosumer_models;
model heat_source_sink

  extends ProsNet.Prosumers.SecondarySides.BaseClasses.ControlVolumeDynParam;

  Modelica.Fluid.Interfaces.FluidPort_a port_hot(redeclare package Medium =
        Media.Water)
    annotation (Placement(transformation(extent={{-70,-110},{-50,-90}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_cold(redeclare package Medium =
        Media.Water)
    annotation (Placement(transformation(extent={{50,-110},{70,-90}})));
  Modelica.Blocks.Interfaces.RealOutput m_dot_sec_is "kg/s" annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={0,-98})));
  Modelica.Blocks.Interfaces.RealOutput T_sec_hot annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-80,-100})));
  Modelica.Blocks.Interfaces.RealOutput T_sec_cold annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={80,-100})));
  Modelica.Fluid.Sensors.TemperatureTwoPort T_sens_hot(redeclare package Medium =
        Media.Water) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-60,-50})));
  Modelica.Fluid.Sensors.TemperatureTwoPort T_sens_cold(redeclare package
      Medium = Media.Water) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={60,-50})));
  Modelica.Fluid.Sensors.MassFlowRate massFlowRate(redeclare package Medium =
               Media.Water) annotation (Placement(
        transformation(
        extent={{-10,10},{10,-10}},
        rotation=90,
        origin={-58,-16})));
  New_Prosumers.SF1_indep_Newcontrol sF1_indep_Newcontrol
    annotation (Placement(transformation(extent={{-36,-24},{48,38}})));
equation
  connect(T_sens_hot.port_a, port_hot)
    annotation (Line(points={{-60,-60},{-60,-100}}, color={0,127,255}));
  connect(T_sens_hot.T, T_sec_hot) annotation (Line(points={{-71,-50},{-80,-50},
          {-80,-100}}, color={0,0,127}));
  connect(T_sens_cold.port_b, port_cold)
    annotation (Line(points={{60,-60},{60,-100}}, color={0,127,255}));
  connect(T_sens_cold.T, T_sec_cold)
    annotation (Line(points={{71,-50},{80,-50},{80,-100}}, color={0,0,127}));
  connect(massFlowRate.port_a, T_sens_hot.port_b)
    annotation (Line(points={{-58,-26},{-58,-34},{-60,-34},{-60,-40}},
                                                   color={0,127,255}));
  connect(massFlowRate.m_flow, m_dot_sec_is) annotation (Line(points={{-47,-16},
          {-34,-16},{-34,-84},{0,-84},{0,-98}}, color={0,0,127}));
  connect(sF1_indep_Newcontrol.port_b1, T_sens_cold.port_a) annotation (Line(
        points={{17.2,47.2},{18,47.2},{18,48},{60,48},{60,-40}}, color={0,127,
          255}));
  connect(sF1_indep_Newcontrol.port_a2, massFlowRate.port_b) annotation (Line(
        points={{12.8,47.2},{-58,47.2},{-58,-6}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,40},{100,-100}},
          lineColor={0,0,0},
          lineThickness=1),
        Polygon(
          points={{100,40},{8,100},{-8,100},{-100,40},{100,40}},
          lineColor={0,0,0},
          lineThickness=1)}),
      Diagram(coordinateSystem(preserveAspectRatio=false)));
end heat_source_sink;