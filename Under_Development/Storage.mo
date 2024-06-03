within ProsNet.Under_Development;
package Storage
  model StratifiedHeatStorage
    "Model of a stratified tank for thermal energy storage"
    extends Buildings.Fluid.Storage.BaseClasses.PartialStratified(vol(each nPorts=3));

    Modelica.Fluid.Interfaces.FluidPorts_a fluPorVol[nSeg](
      redeclare each final package Medium = Medium)
      "Fluid port that connects to the control volumes of the tank"
      annotation (Placement(transformation(extent={{-110,-40},{-90,40}}),
          iconTransformation(extent={{-60,-40},{-40,40}})));
    Modelica.Blocks.Interfaces.RealOutput TempTop
      annotation (Placement(transformation(extent={{70,40},{90,60}}),
          iconTransformation(extent={{70,40},{90,60}})));
    Modelica.Blocks.Interfaces.RealOutput Tempside
      annotation (Placement(transformation(extent={{70,-10},{90,10}}),
          iconTransformation(extent={{70,-10},{90,10}})));
    Modelica.Blocks.Interfaces.RealOutput Tempbot
      annotation (Placement(transformation(extent={{70,-64},{90,-44}}),
          iconTransformation(extent={{70,-64},{90,-44}})));
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
      annotation (Placement(transformation(extent={{56,-82},{76,-62}})));
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor1
      annotation (Placement(transformation(extent={{78,-10},{98,10}})));
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor2
      annotation (Placement(transformation(extent={{62,82},{82,102}})));
    Modelica.Fluid.Interfaces.FluidPorts_a fluPorVol1[size(fluPorVol, 1)]
      "Fluid port that connects to the control volumes of the tank" annotation (
       Placement(transformation(extent={{26,-38},{46,42}}), iconTransformation(
            extent={{26,-38},{46,42}})));
  equation
    connect(port_a, vol[1].ports[1]) annotation (Line(points={{0,100},{-80,100},{
            -80,-20},{16,-20},{16,-16}}, color={0,127,255}));
    connect(vol[nSeg].ports[2], port_b) annotation (Line(points={{16,-16},{20,-16},
            {20,-20},{90,-20},{90,-100},{0,-100}},
                                               color={0,127,255}));
    for i in 1:(nSeg-1) loop
      connect(vol[i].ports[2], vol[i + 1].ports[1]) annotation (Line(points={{16,-16},
              {16,-32},{14,-32},{14,-16},{16,-16}}, color={0,127,255}));
    end for;
    for i in 1:nSeg loop
      connect(fluPorVol[i], vol[i].ports[3]) annotation (Line(points={{-100,0},{
              -88,0},{-88,-36},{16,-36},{16,-16}},
                                color={0,127,255}));
    end for;
    connect(Tempbot, temperatureSensor.T)
      annotation (Line(points={{80,-54},{78,-54},{78,-72},{77,-72}},
                                                    color={0,0,127}));
    connect(heaPorBot, temperatureSensor.port)
      annotation (Line(points={{20,-74},{20,-72},{56,-72}}, color={191,0,0}));
    connect(Tempside, temperatureSensor1.T)
      annotation (Line(points={{80,0},{99,0}},  color={0,0,127}));
    connect(temperatureSensor1.port, heaPorSid)
      annotation (Line(points={{78,0},{56,0}}, color={191,0,0}));
    connect(heaPorTop, temperatureSensor2.port) annotation (Line(points={{20,74},
            {56,74},{56,92},{62,92}}, color={191,0,0}));
    connect(temperatureSensor2.T, TempTop)
      annotation (Line(points={{83,92},{96,92},{96,50},{80,50}},
                                                  color={0,0,127}));
    connect(fluPorVol, fluPorVol1) annotation (Line(points={{-100,0},{10,0},{10,
            2},{36,2}}, color={0,127,255}));
    annotation (
  defaultComponentName="tan",
  Documentation(info="<html>
<p>
This is a model of a stratified storage tank.
</p>
<p>
See the
<a href=\"modelica://Buildings.Fluid.Storage.UsersGuide\">
Buildings.Fluid.Storage.UsersGuide</a>
for more information.
</p>
<p>
For a model with enhanced stratification, use
<a href=\"modelica://Buildings.Fluid.Storage.StratifiedEnhanced\">
Buildings.Fluid.Storage.StratifiedEnhanced</a>.
</p>
</html>",   revisions="<html>
<ul>
<li>
March 7, 2022, by Michael Wetter:<br/>
Set <code>final massDynamics=energyDynamics</code>.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1542\">#1542</a>.
</li>
<li>
June 7, 2018 by Filip Jorissen:<br/>
Copied model from Buildings and update the model accordingly.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/314\">#314</a>.
</li>
<li>
June 1, 2018, by Michael Wetter:<br/>
Refactored model to allow a fluid port in the tank that do not have
the enhanced stratification model.<br/>
This is for <a href=\"https://github.com/lbl-srg/modelica-buildings/issues/1182\">
issue 1182</a>.
</li>
<li>
July 29, 2017, by Michael Wetter:<br/>
Removed medium declaration, which is not needed and inconsistent with
the declaration in the base class.
This is for <a href=\"https://github.com/lbl-srg/modelica-buildings/issues/544\">
issue 544</a>.
</li>
<li>
March 28, 2015, by Filip Jorissen:<br/>
Propagated <code>allowFlowReversal</code> and <code>m_flow_small</code>
and set <code>mSenFac=1</code>.
</li>
<li>
January 26, 2015, by Michael Wetter:<br/>
Renamed
<code>hA_flow</code> to <code>H_a_flow</code>,
<code>hB_flow</code> to <code>H_b_flow</code> and
<code>hVol_flow</code> to <code>H_vol_flow</code>
as they output enthalpy flow rate, and not specific enthalpy.
Made various models <code>protected</code>.
</li>
<li>
January 25, 2015, by Michael Wetter:<br/>
Added <code>final</code> to <code>tau = 0</code> in <code>EnthalpyFlowRate</code>.
These sensors do not need dynamics as the enthalpy flow rate
is used to compute a heat flow which is then added to the volume of the tank.
Thus, if there were high frequency oscillations of small mass flow rates,
then they have a small effect on <code>H_flow</code>, and they are
not used in any control loop. Rather, the oscillations are further damped
by the differential equation of the fluid volume.
</li>
<li>
January 25, 2015, by Filip Jorissen:<br/>
Set <code>tau = 0</code> in <code>EnthalpyFlowRate</code>
sensors for increased simulation speed.
</li>
<li>
August 29, 2014, by Michael Wetter:<br/>
Replaced the use of <code>Medium.lambda_const</code> with
<code>Medium.thermalConductivity(sta_default)</code> as
<code>lambda_const</code> is not declared for all media.
This avoids a translation error if certain media are used.
</li>
<li>
June 18, 2014, by Michael Wetter:<br/>
Changed the default value for the energy balance initialization to avoid
a dependency on the global <code>system</code> declaration.
</li>
<li>
July 29, 2011, by Michael Wetter:<br/>
Removed <code>use_T_start</code> and <code>h_start</code>.
</li>
<li>
February 18, 2011, by Michael Wetter:<br/>
Changed default start values for temperature and pressure.
</li>
<li>
October 25, 2009 by Michael Wetter:<br/>
Changed computation of heat transfer through top (and bottom) of tank. Now,
the thermal resistance of the fluid is not taken into account, i.e., the
top (and bottom) element is assumed to be mixed.
</li>
<li>
October 23, 2009 by Michael Wetter:<br/>
Fixed bug in computing heat conduction of top and bottom segment.
In the previous version,
for computing the heat conduction between the top (or bottom) segment and
the outside,
the whole thickness of the water volume was used
instead of only half the thickness.
</li>
<li>
February 19, 2009 by Michael Wetter:<br/>
Changed declaration that constrains the medium. The earlier
declaration caused the medium model to be not shown in the parameter
window.
</li>
<li>
October 31, 2008 by Michael Wetter:<br/>
Added heat conduction.
</li>
<li>
October 23, 2008 by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),
  Icon(graphics={
          Rectangle(
            extent={{-40,60},{40,20}},
            lineColor={255,0,0},
            fillColor={255,0,0},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-40,-20},{40,-60}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-40,20},{40,-20}},
            lineColor={255,0,0},
            pattern=LinePattern.None,
            fillColor={0,0,127},
            fillPattern=FillPattern.CrossDiag),
          Text(
            extent={{100,106},{134,74}},
            textColor={0,0,127},
            textString="QLoss"),
          Rectangle(
            extent={{-10,10},{10,-10}},
            lineColor={0,0,0},
            fillPattern=FillPattern.Sphere,
            fillColor={255,255,255}),
          Rectangle(
            extent={{50,68},{40,-66}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={255,255,0},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-40,66},{-50,-68}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={255,255,0},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-48,68},{50,60}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={255,255,0},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-48,-60},{50,-68}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={255,255,0},
            fillPattern=FillPattern.Solid),
          Line(
            points={{26,72},{102,72},{100,72}},
            color={127,0,0},
            pattern=LinePattern.Dot),
          Line(
            points={{56,6},{56,72},{58,72}},
            color={127,0,0},
            pattern=LinePattern.Dot),
          Line(
            points={{22,-74},{70,-74},{70,72}},
            color={127,0,0},
            pattern=LinePattern.Dot)}));
  end StratifiedHeatStorage;

  model Battery "Simple model of a battery"
    parameter Modelica.Units.SI.Efficiency etaCha(max=1) = 0.9
      "Efficiency during charging";
    parameter Modelica.Units.SI.Efficiency etaDis(max=1) = 0.9
      "Efficiency during discharging";
   parameter Real SOC_start(min=0, max=1, unit="1")=0.1 "Initial state of charge";
    parameter Modelica.Units.SI.Energy EMax(min=0, displayUnit="kW.h")
      "Maximum available charge";
    parameter Modelica.Units.SI.Voltage V_nominal
      "Nominal voltage (V_nominal >= 0)";
   Modelica.Blocks.Interfaces.RealInput P(unit="W")
      "Power stored in battery (if positive), or extracted from battery (if negative)"
      annotation (Placement(transformation(extent={{-20,-20},{20,20}},
          rotation=270,
          origin={0,108}),
          iconTransformation(extent={{-20,-20},{20,20}},
          rotation=270,
          origin={0,100})));
    Modelica.Blocks.Interfaces.RealOutput SOC(min=0, max=1, unit="1")
      "State of charge"
      annotation (Placement(transformation(extent={{100,50},{120,70}})));
    Buildings.Electrical.DC.Interfaces.Terminal_p terminal "Generalized terminal"
      annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  protected
    Buildings.Electrical.DC.Storage.BaseClasses.Charge cha(
      final EMax=EMax,
      final SOC_start=SOC_start,
      final etaCha=etaCha,
      final etaDis=etaDis) "Charge model"
      annotation (Placement(transformation(extent={{40,50},{60,70}})));
    Buildings.Electrical.DC.Loads.Conductor bat(final mode=Buildings.Electrical.Types.Load.VariableZ_P_input,
        final V_nominal=V_nominal) "Power exchanged with battery pack"
      annotation (Placement(transformation(extent={{40,-10},{60,10}})));
    Modelica.Blocks.Math.Gain gain(final k=-1)
      annotation (Placement(transformation(extent={{22,10},{42,30}})));

  equation
    connect(cha.SOC, SOC)    annotation (Line(
        points={{61,60},{110,60}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(cha.P, P)    annotation (Line(
        points={{38,60},{0,60},{0,108},{8.88178e-16,108}},
        color={0,0,127},
        smooth=Smooth.None));

    connect(bat.terminal, terminal) annotation (Line(
        points={{40,0},{-100,0}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(P, gain.u) annotation (Line(
        points={{8.88178e-16,108},{8.88178e-16,20},{20,20}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(gain.y, bat.Pow) annotation (Line(
        points={{43,20},{68,20},{68,8.88178e-16},{60,8.88178e-16}},
        color={0,0,127},
        smooth=Smooth.None));
    annotation ( Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
          graphics={
          Polygon(
            points={{-62,40},{-62,-40},{72,-40},{72,40},{-62,40}},
            smooth=Smooth.None,
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Polygon(
            points={{58,32},{58,-30},{32,-30},{10,32},{58,32}},
            smooth=Smooth.None,
            pattern=LinePattern.None,
            lineColor={0,0,0},
            fillColor={0,127,0},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-34,32},{-12,-30},{-32,-30},{-54,32},{-34,32}},
            smooth=Smooth.None,
            pattern=LinePattern.None,
            lineColor={0,0,0},
            fillColor={0,127,0},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-2,32},{20,-30},{0,-30},{-22,32},{-2,32}},
            smooth=Smooth.None,
            pattern=LinePattern.None,
            lineColor={0,0,0},
            fillColor={0,127,0},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-74,12},{-74,-12},{-62,-12},{-62,12},{-74,12}},
            smooth=Smooth.None,
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Text(
            extent={{-50,68},{-20,100}},
            textColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid,
            textString="P"),
          Line(
            points={{-74,0},{-100,0},{-100,0}},
            color={0,0,0},
            smooth=Smooth.None),
          Text(
            extent={{-150,70},{-50,20}},
            textColor={0,0,0},
            textString="+"),
          Text(
            extent={{-150,-12},{-50,-62}},
            textColor={0,0,0},
            textString="-"),
          Text(
            extent={{44,70},{100,116}},
            textColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid,
            textString="SOC"),
          Text(
            extent={{44,154},{134,112}},
            textColor={0,0,255},
            textString="%name")}),
      Documentation(info="<html>
<p>
Simple model of a battery.
</p>
<p>
This model takes as an input the power that should be stored in the battery (if <i>P &gt; 0</i>)
or that should be extracted from the battery.
The model uses a fictitious conductance
(see <a href=\"modelica://Buildings.Electrical.DC.Loads.Conductor\">Buildings.Electrical.DC.Loads.Conductor</a>) <i>G</i> such that
<i>P = u &nbsp; i</i> and <i>i = u &nbsp; G,</i> where
<i>u</i> is the voltage difference across the pins and
<i>i</i> is the current at the positive pin.
</p>
<p>
The output connector <code>SOC</code> is the state of charge of the battery.
This model does not enforce that the state of charge is between zero and one.
However, each time the state of charge crosses zero or one, a warning will
be written to the simulation log file.
The model also does not limit the current through the battery. The user should
provide a control so that only a reasonable amount of power is exchanged,
and that the state of charge remains between zero and one.
</p>
</html>", revisions="<html>
<ul>
<li>
December 6, 2021, by Michael Wetter:<br/>
Corrected wrong unit string.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/2798\">issue 2798</a>.
</li>
<li>
September 24, 2015 by Michael Wetter:<br/>
Removed binding of <code>P_nominal</code> as
this parameter is disabled and assigned a value
in the <code>initial equation</code> section.
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/426\">issue 426</a>.
</li>
<li>
March 19, 2015, by Michael Wetter:<br/>
Removed redeclaration of phase system in <code>Terminal_n</code> and
<code>Terminal_p</code> as it is already declared to the be the same
phase system, and it is not declared to be replaceable.
This avoids a translation error in OpenModelica.
</li>
<li>
June 2, 2014, by Marco Bonvini:<br/>
Revised documentation.
</li>
<li>
January 8, 2013, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
  end Battery;
end Storage;
