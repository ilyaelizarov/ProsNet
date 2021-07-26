within ProsNet.Fluid.Pumps.Examples;
model MoverParameter
  "Example model of movers using a parameter for setting the stage"
  extends Modelica.Icons.Example;

  parameter Modelica.Units.SI.MassFlowRate m_flow_nominal=2
    "Nominal mass flow rate";

  ProsNet.Fluid.Pumps.FlowControlled_m_flow pump_m_flow(
    m_flow_nominal=m_flow_nominal,
    use_inputFilter=false,
    massFlowRates={0,0.5,1}*m_flow_nominal,
    inputType=ProsNet.Fluid.Types.InputType.Constant,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    "Pump with m_flow input"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));

  ProsNet.Fluid.Sources.Boundary_pT sou(nPorts=2) "Fluid source"
              annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));

  ProsNet.Fluid.Sources.Boundary_pT sin(nPorts=2) "Fluid sink"
    annotation (Placement(transformation(extent={{80,-10},{60,10}})));
  ProsNet.Fluid.Pumps.SpeedControlled_y pump_y(
    redeclare Data.Pumps.QuadraticCharacteristic per,
    use_inputFilter=false,
    inputType=ProsNet.Fluid.Types.InputType.Constant,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    "Pump with normalised speed input"
    annotation (Placement(transformation(extent={{-8,-50},{12,-30}})));
  parameter Modelica.Units.SI.PressureDifference dp_nominal=10000
    "Nominal pressure raise";
equation
  connect(sou.ports[1], pump_m_flow.port_a) annotation (Line(
      points={{-60,2},{-60,0},{-10,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pump_m_flow.port_b, sin.ports[1]) annotation (Line(
      points={{10,0},{60,0},{60,2}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pump_y.port_b, sin.ports[2])
    annotation (Line(points={{12,-40},{60,-40},{60,-2}},color={0,127,255}));
  connect(pump_y.port_a, sou.ports[2]) annotation (Line(points={{-8,-40},{-60,-40},
          {-60,-2}},           color={0,127,255}));
  annotation (
    __Dymola_Commands(file=
          "modelica://IBPSA/Resources/Scripts/Dymola/Fluid/Movers/Examples/MoverParameter.mos"
        "Simulate and plot"),
    Documentation(info="<html>
<p>
This example demonstrates the use of a <code>Parameter</code>
set point for a mover model.
</p>
</html>", revisions="<html>
<ul>
<li>
February 17, 2016, by Michael Wetter:<br/>
Updated parameter names for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/396\">#396</a>.
</li>
<li>
January 22, 2016, by Michael Wetter:<br/>
Corrected type declaration of pressure difference.
This is
for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/404\">#404</a>.
</li>
<li>
August 24, 2015, by Filip Jorissen:<br/>
First implementation.
</li>
</ul>
</html>"),
    experiment(Tolerance=1e-06, StopTime=1));
end MoverParameter;
