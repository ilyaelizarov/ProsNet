within ProsNet.Fluid.Pipes;
model InsulatedPipe

  extends ProsNet.Fluid.Interfaces.PartialTwoPort(
    redeclare package Medium = ProsNet.Media.Water);

  parameter Modelica.Units.SI.Temperature T_amb(displayUnit="degC") = 285.15
    "Ambient temperature";

  parameter Real R_ins(min=0,unit="(K.m)/W")=3.78 "Thermal resistance per meter";

  parameter Modelica.Units.SI.Length length=10 "Length of the pipe";

  parameter Modelica.Units.SI.Length diameter=0.022 "Diameter of the pipe";

  parameter Real zeta(min=0)=3.5 "Local resistance coefficient";

  parameter Modelica.Fluid.Types.Dynamics energyDynamics = Modelica.Fluid.Types.Dynamics.SteadyState
    "Energy dynamics" annotation(Dialog(tab="Dynamics"));

  parameter Modelica.Units.SI.MassFlowRate m_flow_start=0
    "Start value for mass flow rate" annotation (Dialog(tab="Dynamics"));

  parameter Modelica.Units.SI.Pressure p_start=101325
    "Start value for pressure" annotation (Dialog(tab="Dynamics"));

  parameter Modelica.Units.SI.Temperature T_start=293.15
    "Start value for temperature" annotation (Dialog(tab="Dynamics"));

  Modelica.Fluid.Pipes.DynamicPipe pipe(mediums(each preferredMediumStates = false),
   redeclare replaceable package Medium =
        ProsNet.Media.Water,
    length=length,
    diameter=diameter,
    use_HeatTransfer=true)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  inner Modelica.Fluid.System system(
    energyDynamics=energyDynamics,
    m_flow_start=m_flow_start,
    p_start=p_start,
    T_start=T_start)
    annotation (Placement(transformation(extent={{80,80},{100,100}})));
  Modelica.Fluid.Fittings.SimpleGenericOrifice orifice2(
    redeclare replaceable package Medium = ProsNet.Media.Water,
    diameter=diameter,
    zeta=zeta/2)
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  Modelica.Fluid.Fittings.SimpleGenericOrifice orifice1(
    redeclare replaceable package Medium = ProsNet.Media.Water,
    diameter=diameter,
    zeta=zeta/2)
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor thCond(G=1/R_ins*
        length) annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=90,
        origin={-1,39})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature bou(T=T_amb)
    "Boundary temperature"
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
equation
  connect(port_a, orifice1.port_a)
    annotation (Line(points={{-100,0},{-60,0}}, color={0,127,255}));
  connect(orifice1.port_b, pipe.port_a)
    annotation (Line(points={{-40,0},{-10,0}}, color={0,127,255}));
  connect(pipe.port_b, orifice2.port_a)
    annotation (Line(points={{10,0},{40,0}}, color={0,127,255}));
  connect(orifice2.port_b, port_b)
    annotation (Line(points={{60,0},{100,0}}, color={0,127,255}));
  connect(bou.port, thCond.port_b)
    annotation (Line(points={{-60,70},{-1,70},{-1,46}}, color={191,0,0}));
  connect(thCond.port_a, pipe.heatPorts[1])
    annotation (Line(points={{-1,32},{0.1,32},{0.1,4.4}}, color={191,0,0}));
  annotation (defaultComponentName="pipe", Icon(coordinateSystem(preserveAspectRatio=true), graphics={
                                     Rectangle(
          extent={{-100,44},{100,-44}},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,127,255}),
        Rectangle(
          extent={{-100,60},{100,44}},
          lineColor={28,108,200},
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,-44},{100,-60}},
          lineColor={28,108,200},
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{22,-68},{62,-83},{22,-98},{22,-68}},
          lineColor={0,128,255},
          fillColor={0,128,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{22,-73},{52,-83},{22,-93},{22,-73}},
          lineColor={255,255,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(
          points={{57,-83},{-58,-83}},
          color={0,128,255}),
        Polygon(
          points={{22,-68},{62,-83},{22,-98},{22,-68}},
          lineColor={0,128,255},
          fillColor={0,128,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{22,-73},{52,-83},{22,-93},{22,-73}},
          lineColor={255,255,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(
          points={{57,-83},{-58,-83}},
          color={0,128,255})}),                    Diagram(
        coordinateSystem(preserveAspectRatio=true)),
    Documentation(info="<html>
<p><b>Model of insulated pipe.</b></p>
<p><br>The pipe model is based on <a href=\"modelica://Modelica.Fluid.Pipes.DynamicPipe\">DynamicPipe</a> from the Modelica Standard Library. Two features have been added to the base model. First, thermal losses are modeled. Second, the influence of minor hydraulic losses can be taken into account.</p>
</html>"));
end InsulatedPipe;
