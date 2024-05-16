within ProsNet.Fluid.Pumps;
model Test_Pump_controler "Flow machine with feedback control"
  extends Modelica.Icons.Example;

  package Medium = Buildings.Media.Air;

  parameter Modelica.Units.SI.MassFlowRate m_flow_nominal=0.1
    "Nominal mass flow rate";
  parameter Modelica.Units.SI.PressureDifference dp_nominal=500
    "Nominal pressure difference";

  Buildings.Fluid.Sources.Boundary_pT sou(
    redeclare package Medium = Buildings.Media.Water,
    use_p_in=false,
    p=101325,
    T=293.15,
    nPorts=2) annotation (Placement(transformation(extent={{-82,10},{-62,30}})));
  Buildings.Fluid.Movers.SpeedControlled_y fan(
    redeclare package Medium = Buildings.Media.Water,
      per(pressure(V_flow={0,m_flow_nominal,2*m_flow_nominal}/1.2,
                   dp={2*dp_nominal,dp_nominal,0})),
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Fan"
    annotation (Placement(transformation(extent={{40,40},{60,60}})));
  Buildings.Fluid.Sensors.MassFlowRate senMasFlo(redeclare package Medium =
        Buildings.Media.Water)
    annotation (Placement(transformation(extent={{-40,40},{-20,60}})));
  Buildings.Controls.Continuous.LimPID conPID(
    Td=1,
    k=0.5,
    Ti=15) annotation (Placement(transformation(extent={{0,100},{20,120}})));
  Modelica.Blocks.Math.Gain gain1(k=1/m_flow_nominal)
    annotation (Placement(transformation(extent={{-22,70},{-2,90}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater) annotation (
      Placement(transformation(extent={{-106,-8},{-86,12}}),
        iconTransformation(extent={{-106,-8},{-86,12}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater) annotation (
      Placement(transformation(extent={{94,-8},{114,12}}), iconTransformation(
          extent={{94,-8},{114,12}})));
  Modelica.Blocks.Interfaces.RealInput SetInSignal
    annotation (Placement(transformation(extent={{-102,90},{-62,130}})));
  Modelica.Blocks.Interfaces.RealOutput electrical_power_consumed
    annotation (Placement(transformation(extent={{100,48},{120,68}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p term_p
    annotation (Placement(transformation(extent={{134,28},{154,48}}),
        iconTransformation(extent={{46,-144},{66,-124}})));
  Buildings.Electrical.AC.OnePhase.Loads.Inductive acLoad(mode=Buildings.Electrical.Types.Load.VariableZ_P_input,
      V_nominal=480)
    annotation (Placement(transformation(extent={{9,-9},{-9,9}},
        rotation=0,
        origin={149,59})));
equation
  connect(sou.ports[1], senMasFlo.port_a) annotation (Line(
      points={{-62,19},{-52,19},{-52,50},{-40,50}},
      color={0,127,255}));
  connect(senMasFlo.m_flow, gain1.u) annotation (Line(
      points={{-30,61},{-30,80},{-24,80}},
      color={0,0,127}));
  connect(gain1.y, conPID.u_m) annotation (Line(
      points={{-1,80},{10,80},{10,98}},
      color={0,0,127}));
  connect(conPID.y, fan.y) annotation (Line(
      points={{21,110},{50,110},{50,62}},
      color={0,0,127}));
  connect(fan.port_a, senMasFlo.port_b)
    annotation (Line(points={{40,50},{-20,50}}, color={0,127,255}));
  connect(fan.port_b, port_a) annotation (Line(points={{60,50},{-18,50},{-18,
          2},{-96,2}}, color={0,127,255}));
  connect(sou.ports[2], port_b) annotation (Line(points={{-62,21},{20,21},{20,
          2},{104,2}}, color={0,127,255}));
  connect(conPID.u_s, SetInSignal)
    annotation (Line(points={{-2,110},{-82,110}}, color={0,0,127}));
  connect(fan.P, electrical_power_consumed) annotation (Line(points={{61,59},
          {88,59},{88,58},{110,58}}, color={0,0,127}));
  connect(electrical_power_consumed, acLoad.Pow)
    annotation (Line(points={{110,58},{110,59},{140,59}}, color={0,0,127}));
  connect(acLoad.terminal, term_p) annotation (Line(points={{158,59},{162,59},
          {162,38},{144,38}}, color={0,120,120}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{160,
            160}})),
    Documentation(info="<html>
<p>
This example demonstrates the use of a fan with closed loop control.
The fan is controlled to track a required mass flow rate.
</p>
</html>", revisions="<html>
<ul>
<li>
January 22, 2016, by Michael Wetter:<br/>
Corrected type declaration of pressure difference.
This is
for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/404\">#404</a>.
</li>
<li>
February 14, 2012, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),
    __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Movers/Examples/ClosedLoop_y.mos"
        "Simulate and plot"),
    experiment(
      StopTime=3600,
      Tolerance=1e-06),
    Icon(graphics={
          Polygon(
            points={{-44,-58},{-68,-98},{76,-98},{52,-58},{-44,-58}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillPattern=FillPattern.VerticalCylinder),
          Rectangle(
            extent={{-96,48},{104,-44}},
            fillColor={0,127,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Ellipse(
            extent={{-78,84},{82,-76}},
            fillPattern=FillPattern.Sphere,
            fillColor={0,100,199}),
          Polygon(
            points={{-18,34},{-18,-26},{60,2},{-18,34}},
            pattern=LinePattern.None,
            fillPattern=FillPattern.HorizontalCylinder,
            fillColor={255,255,255})}));
end Test_Pump_controler;
