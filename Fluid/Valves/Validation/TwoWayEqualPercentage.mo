within ProsNet.Fluid.Valves.Validation;
model TwoWayEqualPercentage
  "Two way valve with equal percentage characteristics"
  extends Modelica.Icons.Example;
  package Medium = ProsNet.Media.Water;

    Modelica.Blocks.Sources.Ramp y(
    height=1,
    duration=1,
    offset=0) "Control signal"
                 annotation (Placement(transformation(extent={{-60,90},{-40,110}})));
  ProsNet.Fluid.Sources.Boundary_pT sou(
    redeclare package Medium = Medium,
    nPorts=1,
    use_p_in=false,
    p(displayUnit="Pa") = 306000,
    T=293.15) "Boundary condition for flow source"  annotation (Placement(
        transformation(extent={{-70,-10},{-50,10}})));
  ProsNet.Fluid.Sources.Boundary_pT sin(
    redeclare package Medium = Medium,
    nPorts=1,
    p(displayUnit="Pa") = 3E5,
    T=293.15) "Boundary condition for flow sink"    annotation (Placement(
        transformation(extent={{72,-10},{52,10}})));
  ProsNet.Fluid.Valves.TwoWayEqualPercentage valEqu(
    redeclare package Medium = Medium,
    l=0.05,
    R=10,
    delta0=0.1,
    m_flow_nominal=2,
    use_inputFilter=false,
    dpValve_nominal=6000)
    "Valve model, equal percentage opening characteristics"
    annotation (Placement(transformation(extent={{0,-10},{20,10}})));

equation
  connect(y.y, valEqu.y) annotation (Line(
      points={{-39,100},{-16, 100},{-16, 20},{10,20},{10,12}},
      color={0,0,127}));
  connect(valEqu.port_a, sou.ports[1]) annotation (Line(
      points={{0,0},{-26,0},{-26,0},{-50,0}},
      color={0,127,255}));
  connect(valEqu.port_b, sin.ports[1]) annotation (Line(
      points={{20,0},{36,0},{36,0},{52,0}},
      color={0,127,255}));
    annotation (experiment(Tolerance=1e-6, StopTime=1.0),
__Dymola_Commands(file="modelica://IBPSA/Resources/Scripts/Dymola/Fluid/Actuators/Valves/Examples/TwoWayValves.mos"
        "Simulate and plot"),
    Documentation(info= "<html><head></head><body><p>
Test model for two way valves. Note that the
leakage flow rate has been set to a large value
and the rangeability to a small value
for better visualization of the valve characteristics.
To use common values, use the default values.
</p>
<p>
The parameter <code>filterOpening</code> is set to <code>false</code>,
as this model is used to plot the flow at different opening signals
without taking into account the travel time of the actuator.
</p><p><i>This component was copied from the <a href=\"https://github.com/ibpsa/modelica-ibpsa\">IBPSA library</a>&nbsp;with changes.</i></p>
</body></html>", revisions="<html>
<ul>
<li>
January 29, 2015, by Filip Jorissen:<br/>
Added pressure-independent valve.
</li>
<li>
February 28, 2013, by Michael Wetter:<br/>
Added default value for <code>dpValve_nominal</code>.
</li>
<li>
June 16, 2008 by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),
    Diagram(coordinateSystem(extent={{-100,-100},{100,120}})),
    Icon(coordinateSystem(extent={{-100,-100},{100,120}})));
end TwoWayEqualPercentage;