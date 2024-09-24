within ProsNet.Fluid;
model NETWORK_ideal

  extends ProsNet.Prosumers.SecondarySides.BaseClasses.ControlVolumeDynParam;

  ProsNet.Fluid.HeatExchangers.ControlVolume_T control_volume(
    redeclare final package Medium = ProsNet.Media.Water,
    allowFlowReversal=true,
    m_flow_nominal=0.5,
    tau=tau_cv,
    T_start=T_start_cv,
    energyDynamics=energyDynamics_cv)
    annotation (Placement(transformation(extent={{-28,-26},{28,28}})));
  Modelica.Blocks.Interfaces.RealInput T_set(unit="K", displayUnit="degC")
    "Temperature set point" annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={0,100})));
  Modelica.Fluid.Interfaces.FluidPort_a port_hot(redeclare package Medium =
        ProsNet.Media.Water)
    annotation (Placement(transformation(extent={{-70,-110},{-50,-90}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_cold(redeclare package Medium =
        ProsNet.Media.Water)
    annotation (Placement(transformation(extent={{50,-110},{70,-90}})));
  Modelica.Blocks.Interfaces.RealOutput m_dot_sec_is "kg/s" annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={0,-98})));
  Modelica.Blocks.Interfaces.RealOutput T_prim_hot annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-80,-100})));
  Modelica.Blocks.Interfaces.RealOutput T_prim_cold annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={80,-100})));
  Modelica.Fluid.Sensors.TemperatureTwoPort T_sens_hot(redeclare package Medium =
        ProsNet.Media.Water)
                     annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-60,-50})));
  Modelica.Fluid.Sensors.TemperatureTwoPort T_sens_cold(redeclare package
      Medium = ProsNet.Media.Water)
                            annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={60,-50})));
  Modelica.Fluid.Sensors.MassFlowRate massFlowRate(redeclare package Medium =
               ProsNet.Media.Water)
                            annotation (Placement(
        transformation(
        extent={{-10,10},{10,-10}},
        rotation=90,
        origin={-60,-16})));
  Modelica.Blocks.Math.MultiSum multiSum(nu=2) annotation (Placement(
        transformation(extent={{-96,56},{-84,68}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=-5)
    annotation (Placement(transformation(extent={{-122,26},{-102,46}})));
equation
  connect(T_sens_hot.port_a, port_hot)
    annotation (Line(points={{-60,-60},{-60,-100}}, color={0,127,255}));
  connect(T_sens_hot.T, T_prim_hot) annotation (Line(points={{-71,-50},
          {-80,-50},{-80,-100}}, color={0,0,127}));
  connect(control_volume.port_b, T_sens_cold.port_a)
    annotation (Line(points={{28,1},{60,1},{60,-40}}, color={0,127,255}));
  connect(T_sens_cold.port_b, port_cold)
    annotation (Line(points={{60,-60},{60,-100}}, color={0,127,255}));
  connect(T_sens_cold.T, T_prim_cold) annotation (Line(points={{71,-50},
          {80,-50},{80,-100}}, color={0,0,127}));
  connect(massFlowRate.port_b, control_volume.port_a)
    annotation (Line(points={{-60,-6},{-60,1},{-28,1}}, color={0,127,255}));
  connect(massFlowRate.port_a, T_sens_hot.port_b)
    annotation (Line(points={{-60,-26},{-60,-40}}, color={0,127,255}));
  connect(massFlowRate.m_flow, m_dot_sec_is) annotation (Line(points={{-49,-16},
          {-34,-16},{-34,-84},{0,-84},{0,-98}}, color={0,0,127}));
  connect(multiSum.u[1], T_set) annotation (Line(points={{-96,60.95},
          {-100,60.95},{-100,74},{0,74},{0,100}}, color={0,0,127}));
  connect(realExpression.y, multiSum.u[2]) annotation (Line(points={{
          -101,36},{-98,36},{-98,52},{-100,52},{-100,63.05},{-96,
          63.05}}, color={0,0,127}));
  connect(multiSum.y, control_volume.TSet) annotation (Line(points={{
          -82.98,62},{-44,62},{-44,22.6},{-33.6,22.6}}, color={0,0,
          127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,40},{100,-100}},
          lineColor={0,0,0},
          lineThickness=1),
        Polygon(
          points={{100,40},{100,100},{-100,100},{-100,40},{100,40}},
          lineColor={0,0,0},
          lineThickness=1),
        Text(
          extent={{-116,86},{112,-34}},
          textColor={28,108,200},
          textString="Ideal 

NETWORK")}),
      Diagram(coordinateSystem(preserveAspectRatio=false)));
end NETWORK_ideal;
