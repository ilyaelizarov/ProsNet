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

  package Storage "Package with thermal energy storage models"
    extends Modelica.Icons.VariantsPackage;

    package UsersGuide "User's Guide"
      extends Modelica.Icons.Information;

      annotation (preferredView="info",
      Documentation(info="<html>
<p>
This user's guide describes the storage tank models.
There are three storage tank models in the this package.
</p>
<table summary=\"summary\" border=\"1\" cellspacing=\"0\" cellpadding=\"2\" style=\"border-collapse:collapse;\">
<tr><th>Model name</th>       <th>Description</th>     </tr>
<tr>
<td>
<a href=\"modelica://Buildings.Fluid.Storage.Stratified\">
Buildings.Fluid.Storage.Stratified</a>
</td>
<td>
<p>
This is a model of a stratified storage tank as shown in the figure below.
</p>
<p align=\"center\">
<img alt=\"Image of a storage tank\"
src=\"modelica://Buildings/Resources/Images/Fluid/Storage/Stratified.png\"
width=\"387\" height=\"453\"/>
</p>
<p>
The tank uses several volumes to model the stratification.
Heat conduction is modeled between the volumes through the fluid,
and between the volumes and the ambient.
</p>
<p>
The heat port <code>heaPorVol</code> may be used to connect a temperature sensor
that measures the fluid temperature of an individual volume. It may also
be used to add heat to individual volumes, for example if the tank contains
an electrical resistance heater.
</p>
<p>
Similarly, the fluid port <code>fluPorVol</code> may be used to connect a fluid pipe
to an individual volume. This allows for example to draw water from that volume whose temperature
is close to the temperature required by the consumer.
Conversely, water could be added to that tank volume whose temperature is close to the
inlet water temperature.
If you don't use such a pipe, simply leave the ports unconnected.
</p>
<p>
The tank has <code>nSeg</code> fluid volumes. The top segment has the index <code>1</code>.
Thus, to add a heating element to the bottom element, connect a heat input to
<code>heaPorVol[nSeg]</code>.
</p>
<p>
The heat ports outside the tank insulation can be
used to specify an ambient temperature.
Leave these ports unconnected to force adiabatic boundary conditions.
Note, however, that all heat conduction elements through the tank wall (but not the top and bottom) are connected to the
heat port <code>heaPorSid</code>. Thus, not connecting
<code>heaPorSid</code> means an adiabatic boundary condition in the sense
that <code>heaPorSid.Q_flow = 0</code>. This, however, still allows heat to flow
through the tank walls, modeled by <code>conWal</code>, from one fluid volume
to another one.
</p>
</td>
</tr>
<tr>
<td>
<a href=\"modelica://Buildings.Fluid.Storage.StratifiedEnhanced\">
Buildings.Fluid.Storage.StratifiedEnhanced</a>
</td>
<td>
<p>
The model is identical to
<a href=\"modelica://Buildings.Fluid.Storage.Stratified\">
Buildings.Fluid.Storage.Stratified</a>,
except for the following:
</p>
<ul>
<li>
It adds a correction that reduces the numerical dissipation.
</li>
<li>
It does not contain the fluid ports <code>fluPorVol</code> that
connect from the outside to the individual volumes.
</li>
</ul>
<p>
The correction uses a third order upwind scheme to compute the
outlet temperatures of the segments in the tank. This model
is implemented in
<a href=\"modelica://Buildings.Fluid.Storage.BaseClasses.ThirdOrderStratifier\">
Buildings.Fluid.Storage.BaseClasses.ThirdOrderStratifier</a>.
</p>
</td>
</tr>
<tr>
<td>
<a href=\"modelica://Buildings.Fluid.Storage.StratifiedEnhancedInternalHex\">
Buildings.Fluid.Storage.StratifiedEnhancedInternalHex</a>
</td>
<td>
<p>
This model is identical to
<a href=\"modelica://Buildings.Fluid.Storage.StratifiedEnhanced\">
Buildings.Fluid.Storage.StratifiedEnhanced</a>
except that it adds a heat exchanger to the tank.
</p>
<p>
The modifications consist of adding a heat exchanger
and fluid ports to connect to the heat exchanger.
The modifications allow to run a fluid through the tank causing heat transfer to the stored fluid.
A typical example is a storage tank in a solar hot water system.
</p>
<p>
The heat exchanger model assumes flow through the inside of a helical coil heat exchanger,
and stagnant fluid on the outside. Parameters are used to describe the
heat transfer on the inside of the heat exchanger at nominal conditions, and
geometry of the outside of the heat exchanger. This information is used to compute
an <i>hA</i>-value for each side of the coil.
Convection calculations are then performed to identify heat transfer
between the heat transfer fluid and the fluid in the tank.
</p>
<p>
The location of the heat exchanger can be parameterized as follows:
The parameters <code>hHex_a</code> and <code>hHex_b</code> are the heights
of the heat exchanger ports <code>portHex_a</code> and <code>portHex_b</code>,
measured from the bottom of the tank.
For example, to place the port <code>portHex_b</code> at the bottom of the tank,
set <code>hHexB_b=0</code>.
The parameters <code>hHex_a</code> and <code>hHex_b</code> are then used to provide
a default value for the parameters
<code>segHex_a</code> and <code>segHex_b</code>, which are the numbers of the tank
segments to which the heat exchanger ports <code>portHex_a</code> and <code>portHex_b</code>
are connected.
</p>
<p align=\"center\">
<img alt=\"Image of a storage tank\"
src=\"modelica://Buildings/Resources/Images/Fluid/Storage/StratifiedHex.png\"
width=\"458\" height=\"456\"/>
</p>
<p>
Optionally, this model computes a dynamic response of the heat exchanger.
This can be configured using the parameters
<code>energyDynamicsHexSolid</code>,
<code>energyDynamicsHex</code> and
<code>massDynamicsHex</code>.
For this computation, the fluid volume inside the heat exchanger
and the heat capacity of the heat
exchanger wall <code>CHex</code> are approximated.
Both depend on the length <code>lHex</code>
of the heat exchanger.
The model provides default values for these
parameters, as well as for the heat exchanger material which is
assumed to be steel. These default values can be overwritten by the user.
The default values for the heat exchanger geometry are computed assuming
that there is a cylindrical heat exchanger
made of steel whose diameter is half the diameter of the tank, e.g.,
<i>r<sub>Hex</sub>=r<sub>Tan</sub>/2</i>.
Hence, the length of the heat exchanger is approximated as
<i>l<sub>Hex</sub> = 2 r<sub>Hex</sub> &pi; h = 2 r<sub>Tan</sub>/2 &pi; h</i>,
where <i>h</i> is the distance between the heat exchanger inlet and outlet.
The wall thickness is assumed to be <i>10%</i> of the heat exchanger
outer diameter.
For typical applications, users do not need to change these values.
</p>
<p>
Setting <code>energyDynamicsHexSolid</code> to a dynamic balance and
<code>energyDynamicsHex</code> to a steady-state balance may be of interest
to remove very fast dynamics of the fluid, while still modeling slower
dynamics that arises from the metal of the heat exchanger.
By default, <code>energyDynamicsHexSolid</code> is set
to the same value as <code>energyDynamicsHex</code>
as this seems to be the typical configuration.
</p>
<p>
The heat exchanger is implemented in
<a href=\"modelica://Buildings.Fluid.Storage.BaseClasses.IndirectTankHeatExchanger\">
Buildings.Fluid.Storage.BaseClasses.IndirectTankHeatExchanger</a>.
</p>
</td>
</tr>
</table>
</html>"));
    end UsersGuide;

    model ExpansionVessel "Expansion vessel with fixed pressure"
     extends ProsNet.Fluid.Building_Fluid.Interfaces.LumpedVolumeDeclarations(
        final energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        final massDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        final mSenFac=1);
      parameter Modelica.Units.SI.Volume V_start(start=1)
        "Volume of liquid stored in the vessel at the start of the simulation";

      Modelica.Fluid.Interfaces.FluidPort_a port_a(
        redeclare package Medium = Medium) "Fluid port"
        annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
      Modelica.Units.SI.Mass m "Mass of liquid in the vessel";

    protected
      final parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(
          T=T_start,
          p=p_start,
          X=X_start[1:Medium.nXi]) "Medium state at start values";
      final parameter Modelica.Units.SI.Density rho_start=Medium.density(state=
          state_start) "Density, used to compute start and guess values";

      Modelica.Units.SI.Energy H "Internal energy of fluid";
      Modelica.Units.SI.Mass[Medium.nXi] mXi
        "Masses of independent components in the fluid";
      Modelica.Units.SI.Mass[Medium.nC] mC
        "Masses of trace substances in the fluid";
      Medium.ExtraProperty C[Medium.nC](nominal=C_nominal)
        "Trace substance mixture content";

    initial equation
      m = V_start * rho_start;
      H = m*Medium.specificInternalEnergy(
              Medium.setState_pTX(p=p_start, T=T_start, X= X_start[1:Medium.nXi]));
      mXi = m*X_start[1:Medium.nXi];
      mC = m*C_start[1:Medium.nC];
    equation
      assert(m > 1.0E-8,
        "Expansion vessel is undersized. You need to increase the value of the parameter V_start.");
      // Conservation equations
      der(m)   = port_a.m_flow;
      der(H)   = port_a.m_flow * actualStream(port_a.h_outflow);
      der(mXi) = port_a.m_flow * actualStream(port_a.Xi_outflow);
      der(mC)  = port_a.m_flow * actualStream(port_a.C_outflow);
      // Properties of outgoing flow.
      // The port pressure is set to a constant value.
      port_a.p          = p_start;
      m*port_a.h_outflow  = H;
      m*port_a.Xi_outflow = mXi;
      m*port_a.C_outflow  = mC;

       annotation (Icon(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                -100},{100,100}}), graphics={
            Text(
              extent={{-148,98},{152,138}},
              textString="%name",
              textColor={0,0,255}),
            Rectangle(
              extent={{-80,80},{80,-80}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-68,70},{70,-70}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-68,18},{-68,18},{-54,32},{-28,16},{0,30},{26,16},{46,32},{
                  70,18},{70,18},{70,-70},{70,-70},{-68,-70},{-68,-70},{-68,18}},
              lineColor={0,0,255},
              smooth=Smooth.Bezier,
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{2,-80},{-2,-90}},
              lineColor={0,0,255},
              pattern=LinePattern.None,
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid)}),
    defaultComponentName="exp",
    Documentation(info="<html>
<p>
This is a model of a pressure expansion vessel. The vessel has a constant pressure
that is equal to the value of the parameter <code>p_start</code>.
The model takes into account the energy and mass balance of the medium.
It has no heat exchange with the ambient.
</p>
<p>
The expansion vessel needs to be used in closed loops that contain
water to set a reference pressure and, for liquids where the
density is modeled as a function of temperature, to allow for
the thermal expansion of the liquid.
</p>
<p>
Note that alternatively, the model
<a href=\"modelica://Buildings.Fluid.Sources.Boundary_pT\">
Buildings.Fluid.Sources.Boundary_pT</a> may be used to set
a reference pressure. The main difference between these two models
is that in this model, there is an energy and mass balance for the volume.
In contrast, for
<a href=\"modelica://Buildings.Fluid.Sources.Boundary_pT\">
Buildings.Fluid.Sources.Boundary_pT</a>,
any mass flow rate that flows out of the model will be at a user-specified temperature.
Therefore, <a href=\"modelica://Buildings.Fluid.Sources.Boundary_pT\">
Buildings.Fluid.Sources.Boundary_pT</a> leads to smaller systems
of equations, which may result in faster simulation.
</p>
</html>",     revisions="<html>
<ul>
<li>
May 11, 2022, by Michael Wetter:<br/>
Removed nonused parameter <code>p</code>.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1614\">IBPSA, #1614</a>.
</li>
<li>
May 29, 2014, by Michael Wetter:<br/>
Removed undesirable annotation <code>Evaluate=true</code>.
</li>
<li>
March 25, 2014 by Michael Wetter:<br/>
Revised the model to use a constant pressure rather than a constant volume of
water and gas. This leads to a simpler model.
</li>
<li>
August 1, 2013 by Michael Wetter:<br/>
Updated model to use new connector <code>mWat_flow</code>.
</li>
<li>
February 7, 2012 by Michael Wetter:<br/>
Revised due to changes in conservation equations in <code>Buildings.Fluid.Interfaces</code>.
</li>
<li>
September 16, 2011 by Michael Wetter:<br/>
Set <code>m(stateSelect=StateSelect.always)</code>, since
setting the <code>stateSelect</code> attribute leads to smaller systems of equations.
</li>
<li>
July 26, 2011 by Michael Wetter:<br/>
Revised model to use new declarations from
<a href=\"modelica://Buildings.Fluid.Interfaces.LumpedVolumeDeclarations\">
Buildings.Fluid.Interfaces.LumpedVolumeDeclarations</a>.
</li>
<li>
May 25, 2011 by Michael Wetter:<br/>
Revised model due to a change in the fluid volume model.
</li>
<li>
Nov. 4, 2009 by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
    end ExpansionVessel;

    model Stratified "Model of a stratified tank for thermal energy storage"
      extends
        ProsNet.Under_Development.Storage.Storage.BaseClasses.PartialStratified(
          vol(each nPorts=3));

      Modelica.Fluid.Interfaces.FluidPorts_a fluPorVol[nSeg](
        redeclare each final package Medium = Medium)
        "Fluid port that connects to the control volumes of the tank"
        annotation (Placement(transformation(extent={{-110,-40},{-90,40}}),
            iconTransformation(extent={{-60,-40},{-40,40}})));
    equation
      connect(port_a, vol[1].ports[1]) annotation (Line(points={{0,100},{-80,
              100},{-80,-20},{16,-20},{16,-16}},
                                           color={0,127,255}));
      connect(vol[nSeg].ports[2], port_b) annotation (Line(points={{16,-16},{20,
              -16},{20,-20},{90,-20},{90,-100},{0,-100}},
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
</html>",     revisions="<html>
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
    end Stratified;

    model StratifiedEnhanced "Stratified tank model with enhanced discretization"
      extends BaseClasses.PartialStratified(
        nSeg=4,
        vol(each prescribedHeatFlowRate=true,
            each nPorts=3));

    protected
      ProsNet.Fluid.Building_Fluid.Sensors.EnthalpyFlowRate H_a_flow(
        redeclare package Medium = Medium,
        final m_flow_nominal=m_flow_nominal,
        final tau=0,
        final allowFlowReversal=allowFlowReversal,
        final m_flow_small=m_flow_small) "Enthalpy flow rate at port a"
        annotation (Placement(transformation(extent={{-60,-90},{-40,-70}})));
      ProsNet.Fluid.Building_Fluid.Sensors.EnthalpyFlowRate[nSeg - 1]
        H_vol_flow(
        redeclare package Medium = Medium,
        each final m_flow_nominal=m_flow_nominal,
        each final tau=0,
        each final allowFlowReversal=allowFlowReversal,
        each final m_flow_small=m_flow_small)
        "Enthalpy flow rate between the volumes"
        annotation (Placement(transformation(extent={{-20,-50},{0,-30}})));
      ProsNet.Fluid.Building_Fluid.Sensors.EnthalpyFlowRate H_b_flow(
        redeclare package Medium = Medium,
        final m_flow_nominal=m_flow_nominal,
        final tau=0,
        final allowFlowReversal=allowFlowReversal,
        final m_flow_small=m_flow_small) "Enthalpy flow rate at port b"
        annotation (Placement(transformation(extent={{50,-90},{70,-70}})));

      BaseClasses.ThirdOrderStratifier str(
        redeclare package Medium = Medium,
        nSeg=nSeg,
        m_flow_small=m_flow_small) "Model to reduce numerical dissipation"
        annotation (Placement(transformation(extent={{-60,-50},{-40,-30}})));

      Modelica.Blocks.Sources.RealExpression mTan_flow(y=port_a.m_flow)
        "Mass flow rate at port a" annotation (Placement(transformation(extent={{-94,-42},
                {-74,-22}})));
    equation
      connect(H_a_flow.port_b, vol[1].ports[1]) annotation (Line(points={{-40,-80},
              {-40,-80},{14,-80},{14,-16},{16,-16}},color={0,127,255}));
      connect(vol[nSeg].ports[2], H_b_flow.port_a) annotation (Line(points={{16,-16},
              {14,-16},{14,-80},{50,-80}}, color={0,127,255}));
      connect(H_b_flow.port_b, port_b) annotation (Line(points={{70,-80},{80,-80},{
              80,-100},{0,-100}}, color={0,127,255}));
      for i in 1:(nSeg-1) loop
        connect(vol[i].ports[2], H_vol_flow[i].port_a) annotation (Line(points={{16,
                -16},{16,-20},{-28,-20},{-28,-40},{-20,-40}}, color={0,127,255}));
        connect(H_vol_flow[i].port_b, vol[i + 1].ports[1]) annotation (Line(points={{0,-40},
                {4,-40},{4,-16},{16,-16}},         color={0,127,255}));
      end for;
      connect(port_a, H_a_flow.port_a) annotation (Line(points={{0,100},{-96,100},{
              -96,-80},{-60,-80}}, color={0,127,255}));

      connect(vol[1:nSeg].ports[3], str.fluidPort[2:nSeg+1])
       annotation (Line(points={{16,-16},{16,-18},{-66,-18},{-66,-40},{-60,-40}},
                     color={0,127,255}));
      connect(H_a_flow.H_flow, str.H_flow[1]) annotation (Line(points={{-50,-69},{-50,
              -62},{-68,-62},{-68,-48},{-62,-48}},
                                              color={0,0,127}));
      connect(H_vol_flow[1:nSeg-1].H_flow, str.H_flow[2:nSeg])   annotation (Line(
            points={{-10,-29},{-10,-26},{-24,-26},{-24,-62},{-68,-62},{-68,-48},{
              -62,-48}},                                                color={0,0,
              127}));
      connect(H_b_flow.H_flow, str.H_flow[nSeg + 1]) annotation (Line(points={{60,-69},
              {60,-62},{-52,-62},{-68,-62},{-68,-48},{-62,-48}},
                                                      color={0,0,127}));
      connect(str.heatPort, vol.heatPort)    annotation (Line(points={{-40,-40},{
              -32,-40},{-32,10},{6,10},{6,-6}},      color={191,0,0}));
      connect(port_a, str.fluidPort[1]) annotation (Line(points={{0,100},{0,92},{
              -72,92},{-72,-40},{-60,-40}},          color={0,127,255}));
      connect(port_b, str.fluidPort[nSeg + 2]) annotation (Line(points={{0,-100},{
              -72,-100},{-72,-40},{-60,-40}},
                               color={0,127,255}));
      connect(mTan_flow.y, str.m_flow) annotation (Line(points={{-73,-32},{-68.5,
              -32},{-68.5,-31.8},{-62,-31.8}}, color={0,0,127}));
      annotation (
    defaultComponentName="tan",
    Documentation(info="<html>
<p>
This is a model of a stratified storage tank for thermal energy storage.
</p>
<p>
See the
<a href=\"modelica://Buildings.Fluid.Storage.UsersGuide\">
Buildings.Fluid.Storage.UsersGuide</a>
for more information.
</p>
<h4>Limitations</h4>
<p>
The model requires at least 4 fluid segments. Hence, set <code>nSeg</code> to 4 or higher.
</p>
</html>",     revisions="<html>
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
March 29, 2012 by Wangda Zuo:<br/>
Revised the implementation to reduce the temperature overshoot.
</li>
<li>
June 23, 2010 by Michael Wetter and Wangda Zuo:<br/>
Changed model that is used to correct the numerical diffusion.
The previous version used the model from Stefan Wischhusen,
<a href=\"http://www.modelica.org/events/modelica2006/Proceedings/sessions/Session3a2.pdf\">
An Enhanced Discretization Method for Storage
Tank Models within Energy Systems</a>,
<i>Modelica Conference</i>,
Vienna, Austria, September 2006. However, for situations where there is a strong stratification,
this model can lead to a large overshoot in tank temperatures, leading to a violation of the
second law.
In this revision, the model that computes the volume outlet temperatures has been changed to a third order upwind scheme,
which is implemented in
<a href=\"modelica://Buildings.Fluid.Storage.BaseClasses.ThirdOrderStratifier\">
Buildings.Fluid.Storage.BaseClasses.ThirdOrderStratifier</a>.
</li>
<li>
October 23, 2008 by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),
     Icon(graphics={
            Rectangle(
              extent={{-40,20},{40,-20}},
              lineColor={0,0,255},
              pattern=LinePattern.None,
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-10,10},{10,-10}},
              lineColor={0,0,0},
              fillPattern=FillPattern.Sphere,
              fillColor={255,255,255}),
            Rectangle(
              extent={{-40,68},{-50,-66}},
              lineColor={0,0,255},
              pattern=LinePattern.None,
              fillColor={255,255,0},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{50,68},{40,-66}},
              lineColor={0,0,255},
              pattern=LinePattern.None,
              fillColor={255,255,0},
              fillPattern=FillPattern.Solid)}));
    end StratifiedEnhanced;

    model StratifiedEnhancedInternalHex
      "A model of a water storage tank with a secondary loop and intenral heat exchanger"
      extends StratifiedEnhanced;

      replaceable package MediumHex =
          Modelica.Media.Interfaces.PartialMedium "Medium in the heat exchanger"
        annotation(Dialog(tab="General", group="Heat exchanger"));

      parameter Modelica.Units.SI.Height hHex_a
        "Height of portHex_a of the heat exchanger, measured from tank bottom"
        annotation (Dialog(tab="General", group="Heat exchanger"));

      parameter Modelica.Units.SI.Height hHex_b
        "Height of portHex_b of the heat exchanger, measured from tank bottom"
        annotation (Dialog(tab="General", group="Heat exchanger"));

      parameter Integer hexSegMult(min=1) = 2
        "Number of heat exchanger segments in each tank segment"
        annotation(Dialog(tab="General", group="Heat exchanger"));

      parameter Modelica.Units.SI.Diameter dExtHex=0.025
        "Exterior diameter of the heat exchanger pipe"
        annotation (Dialog(group="Heat exchanger"));

      parameter Modelica.Units.SI.HeatFlowRate Q_flow_nominal
        "Heat transfer at nominal conditions"
        annotation (Dialog(tab="General", group="Heat exchanger"));
      parameter Modelica.Units.SI.Temperature TTan_nominal
        "Temperature of fluid inside the tank at nominal heat transfer conditions"
        annotation (Dialog(tab="General", group="Heat exchanger"));
      parameter Modelica.Units.SI.Temperature THex_nominal
        "Temperature of fluid inside the heat exchanger at nominal heat transfer conditions"
        annotation (Dialog(tab="General", group="Heat exchanger"));
      parameter Real r_nominal(min=0, max=1)=0.5
        "Ratio between coil inside and outside convective heat transfer at nominal heat transfer conditions"
              annotation(Dialog(tab="General", group="Heat exchanger"));

      parameter Modelica.Units.SI.MassFlowRate mHex_flow_nominal
        "Nominal mass flow rate through the heat exchanger"
        annotation (Dialog(group="Heat exchanger"));

      parameter Modelica.Units.SI.PressureDifference dpHex_nominal(displayUnit="Pa")=
           2500 "Pressure drop across the heat exchanger at nominal conditions"
        annotation (Dialog(group="Heat exchanger"));

      parameter Boolean computeFlowResistance=true
        "=true, compute flow resistance. Set to false to assume no friction"
        annotation (Dialog(tab="Flow resistance heat exchanger"));

      parameter Boolean from_dp=false
        "= true, use m_flow = f(dp) else dp = f(m_flow)"
        annotation (Dialog(tab="Flow resistance heat exchanger"));

      parameter Boolean linearizeFlowResistance=false
        "= true, use linear relation between m_flow and dp for any flow rate"
        annotation (Dialog(tab="Flow resistance heat exchanger"));

      parameter Real deltaM=0.1
        "Fraction of nominal flow rate where flow transitions to laminar"
        annotation (Dialog(tab="Flow resistance heat exchanger"));

      parameter Modelica.Fluid.Types.Dynamics energyDynamicsHex=
        Modelica.Fluid.Types.Dynamics.DynamicFreeInitial
        "Formulation of energy balance for heat exchanger internal fluid mass"
        annotation(Evaluate=true, Dialog(tab = "Dynamics heat exchanger", group="Conservation equations"));

      parameter Modelica.Fluid.Types.Dynamics energyDynamicsHexSolid=energyDynamicsHex
        "Formulation of energy balance for heat exchanger solid mass"
        annotation(Evaluate=true, Dialog(tab = "Dynamics heat exchanger", group="Conservation equations"));

      parameter Modelica.Units.SI.Length lHex=rTan*abs(segHex_a - segHex_b)*
          Modelica.Constants.pi "Approximate length of the heat exchanger"
        annotation (Dialog(tab="Dynamics heat exchanger", group="Conservation equations"));

      parameter Modelica.Units.SI.Area ACroHex=(dExtHex^2 - (0.8*dExtHex)^2)*
          Modelica.Constants.pi/4 "Cross sectional area of the heat exchanger"
        annotation (Dialog(tab="Dynamics heat exchanger", group="Conservation equations"));

      parameter Modelica.Units.SI.SpecificHeatCapacity cHex=490
        "Specific heat capacity of the heat exchanger material"
        annotation (Dialog(tab="Dynamics heat exchanger", group="Conservation equations"));

      parameter Modelica.Units.SI.Density dHex=8000
        "Density of the heat exchanger material"
        annotation (Dialog(tab="Dynamics heat exchanger", group="Conservation equations"));

      parameter Modelica.Units.SI.HeatCapacity CHex=ACroHex*lHex*dHex*cHex
        "Capacitance of the heat exchanger without the fluid"
        annotation (Dialog(tab="Dynamics heat exchanger", group="Conservation equations"));
      parameter Boolean allowFlowReversalHex = true
        "= true to allow flow reversal in heat exchanger, false restricts to design direction (portHex_a -> portHex_b)"
        annotation(Dialog(tab="Assumptions", group="Heat exchanger"), Evaluate=true);
      Modelica.Fluid.Interfaces.FluidPort_a portHex_a(
        redeclare final package Medium =MediumHex,
         m_flow(min=if allowFlowReversalHex then -Modelica.Constants.inf else 0))
        "Heat exchanger inlet"
       annotation (Placement(transformation(extent={{-150,-30},{-130,-10}}),
                       iconTransformation(extent={{-110,-48},{-90,-28}})));
      Modelica.Fluid.Interfaces.FluidPort_b portHex_b(
         redeclare final package Medium = MediumHex,
         m_flow(max=if allowFlowReversalHex then Modelica.Constants.inf else 0))
        "Heat exchanger outlet"
       annotation (Placement(transformation(extent={{-150,-90},{-130,-70}}),
            iconTransformation(extent={{-110,-90},{-90,-70}})));

      BaseClasses.IndirectTankHeatExchanger indTanHex(
        redeclare final package MediumTan = Medium,
        redeclare final package MediumHex = MediumHex,
        final nSeg=nSegHex,
        final CHex=CHex,
        final volHexFlu=volHexFlu,
        final Q_flow_nominal=Q_flow_nominal,
        final TTan_nominal=TTan_nominal,
        final THex_nominal=THex_nominal,
        final r_nominal=r_nominal,
        final dExtHex=dExtHex,
        final dp_nominal=dpHex_nominal,
        final m_flow_nominal=mHex_flow_nominal,
        final energyDynamics=energyDynamicsHex,
        final energyDynamicsSolid=energyDynamicsHexSolid,
        final computeFlowResistance=computeFlowResistance,
        from_dp=from_dp,
        final linearizeFlowResistance=linearizeFlowResistance,
        final deltaM=deltaM,
        final allowFlowReversal=allowFlowReversalHex,
        final m_flow_small=1e-4*abs(mHex_flow_nominal))
        "Heat exchanger inside the tank"
         annotation (Placement(transformation(
            extent={{10,-15},{-10,15}},
            rotation=180,
            origin={-120,-20})));

      Modelica.Units.SI.HeatFlowRate QHex_flow=-sum(indTanHex.port.Q_flow)
        "Heat transferred from the heat exchanger to the tank";
    protected
      final parameter Integer segHex_a = nSeg-integer(hHex_a/segHeight)
        "Tank segment in which port a1 of the heat exchanger is located in"
        annotation(Evaluate=true, Dialog(group="Heat exchanger"));

      final parameter Integer segHex_b = nSeg-integer(hHex_b/segHeight)
        "Tank segment in which port b1 of the heat exchanger is located in"
        annotation(Evaluate=true, Dialog(group="Heat exchanger"));

      final parameter Modelica.Units.SI.Height segHeight=hTan/nSeg
        "Height of each tank segment (relative to bottom of same segment)";

      final parameter Modelica.Units.SI.Length dHHex=abs(hHex_a - hHex_b)
        "Vertical distance between the heat exchanger inlet and outlet";

      final parameter Modelica.Units.SI.Volume volHexFlu=Modelica.Constants.pi*(0.8
          *dExtHex)^2/4*lHex "Volume of the heat exchanger";

      final parameter Integer nSegHexTan=
        if segHex_a > segHex_b then segHex_a-segHex_b + 1 else segHex_b-segHex_a + 1
        "Number of tank segments the heat exchanger resides in";

      final parameter Integer nSegHex = nSegHexTan*hexSegMult
        "Number of heat exchanger segments";

    initial equation
      assert(hHex_a >= 0 and hHex_a <= hTan,
        "The parameter hHex_a is outside its valid range.");

      assert(hHex_b >= 0 and hHex_b <= hTan,
        "The parameter hHex_b is outside its valid range.");

      assert(dHHex > 0,
        "The parameters hHex_a and hHex_b must not be equal.");
    equation
      for j in 0:nSegHexTan-1 loop
        for i in 1:hexSegMult loop
          connect(indTanHex.port[j*hexSegMult+i], heaPorVol[segHex_a + (if hHex_a > hHex_b then j else -j)])
            annotation (Line(
           points={{-120,-10.2},{-120,0},{0,0}},
           color={191,0,0},
           smooth=Smooth.None));
        end for;
      end for;
      connect(portHex_a, indTanHex.port_a) annotation (Line(
          points={{-140,-20},{-130,-20}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(indTanHex.port_b, portHex_b) annotation (Line(
          points={{-110,-20},{-100,-20},{-100,-80},{-140,-80}},
          color={0,127,255},
          smooth=Smooth.None));

      annotation (Line(
          points={{-73.2,69},{-70,69},{-70,28},{-16,28},{-16,-2.22045e-16},{0,-2.22045e-16}},
          color={191,0,0},
          smooth=Smooth.None), Icon(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-100},{100,100}}), graphics={
            Rectangle(
              extent={{-94,-38},{28,-42}},
              lineColor={255,0,0},
              fillColor={255,0,0},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-70,-50},{28,-54}},
              lineColor={0,0,255},
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-94,-78},{-68,-82}},
              lineColor={0,0,255},
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-72,-50},{-68,-80}},
              lineColor={0,0,255},
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-4,-48},{28,-50}},
              lineColor={0,0,255},
              fillPattern=FillPattern.Solid,
              fillColor={0,0,255}),
            Rectangle(
              extent={{-4,-42},{28,-46}},
              lineColor={255,0,0},
              fillColor={255,0,0},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-22,-44},{-2,-48}},
              pattern=LinePattern.None,
              fillColor={255,85,85},
              fillPattern=FillPattern.Solid)}),
    defaultComponentName = "tan",
    Documentation(info = "<html>
<p>
This is a model of a stratified storage tank for thermal energy storage with built-in heat exchanger.
</p>
<p>
See the
<a href=\"modelica://Buildings.Fluid.Storage.UsersGuide\">
Buildings.Fluid.Storage.UsersGuide</a>
for more information.
</p>
<h4>Limitations</h4>
<p>
The model requires at least 4 fluid segments. Hence, set <code>nSeg</code> to 4 or higher.
</p>
</html>",
    revisions="<html>
<ul>
<li>
March 7, 2022, by Michael Wetter:<br/>
Removed <code>massDynamics</code> and <code>massDynamicsHex</code>.<br/>
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
June 23, 2016, by Michael Wetter:<br/>
Corrected computation of the heat exchanger location which was wrong
if <code>hHex_a &lt; hHex_b</code>, e.g., the port a of the heat exchanger
is below the port b.
This closes
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/531\">issue 531</a>.
</li>
<li>
January 22, 2016, by Michael Wetter:<br/>
Corrected type declaration of pressure difference.
This is
for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/404\">#404</a>.
</li>
<li>
July 2, 2015, by Michael Wetter:<br/>
Set the default value <code>energyDynamicsHexSolid=energyDynamicsHex</code>
rather than
<code>energyDynamicsHexSolid=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial</code>
as users are not likely to want different settings.
</li>
<li>
July 1, 2015, by Filip Jorissen:<br/>
Added parameter <code>energyDynamicsHexSolid</code>.
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/434\">
#434</a>.
</li>
<li>
March 28, 2015, by Filip Jorissen:<br/>
Propagated <code>allowFlowReversal</code> and <code>m_flow_small</code>.
</li>
<li>
September 2, 2014 by Michael Wetter:<br/>
Replaced the <code>abs()</code> function in the assignment of the parameter
<code>nSegHexTan</code> as the return value of <code>abs()</code>
is a <code>Real</code> which causes a type error during model check.
</li>
<li>
August 29, 2014 by Michael Wetter:<br/>
Corrected issue <a href=\"https://github.com/lbl-srg/modelica-buildings/issues/271\">#271</a>
which led to a compilation error if the heat exchanger and the tank
had different media.
</li>
<li>
April 18, 2014 by Michael Wetter:<br/>
Added missing ceiling function in computation of <code>botHexSeg</code>.
Without this function, this parameter can take on zero, which is wrong
because the Modelica uses one-based arrays.

Revised the model as the old version required the port<sub>a</sub>
of the heat exchanger to be located higher than port<sub>b</sub>.
This makes sense if the heat exchanger is used to heat up the tank,
but not if it is used to cool down a tank, such as in a cooling plant.
The following parameters were changed:
<ol>
<li>Changed <code>hexTopHeight</code> to <code>hHex_a</code>.</li>
<li>Changed <code>hexBotHeight</code> to <code>hHex_b</code>.</li>
<li>Changed <code>topHexSeg</code> to <code>segHex_a</code>,
 and made it protected as this is deduced from <code>hHex_a</code>.</li>
<li>Changed <code>botHexSeg</code> to <code>segHex_b</code>,
 and made it protected as this is deduced from <code>hHex_b</code>.</li>
</ol>
The names of the following ports have been changed:
<ol>
<li>Changed <code>port_a1</code> to <code>portHex_a</code>.</li>
<li>Changed <code>port_b1</code> to <code>portHex_b</code>.</li>
</ol>
The conversion script should update old instances of
this model automatically in Dymola for all of the above changes.
</li>
<li>
May 10, 2013 by Michael Wetter:<br/>
Removed <code>m_flow_nominal_tank</code> which was not used.
</li>
<li>
January 29, 2013 by Peter Grant:<br/>
First implementation.
</li>
</ul>
</html>"),
        Diagram(coordinateSystem(extent={{-140,-100},{100,100}})));
    end StratifiedEnhancedInternalHex;

    package Ice "Ice tank model"
      extends Modelica.Icons.Package;

      model ControlledTank
        "Ice tank with performance based on performance curves and built-in control for outlet temperature"
        extends ProsNet.Under_Development.Storage.Storage.Ice.Tank(limQ_flow(y=
                if tanHeaTra.canMelt.y then m_flow*cp*(max(per.TFre, TSet) -
                TIn.y) elseif tanHeaTra.canFreeze.y then m_flow*cp*(max(TIn.y,
                min(per.TFre, TSet)) - TIn.y) else m_flow*cp*(per.TFre - TIn.y)));

        Modelica.Blocks.Interfaces.RealInput TSet(
          final unit="K",
          final displayUnit="degC")
          "Outlet temperature setpoint during discharging"
          annotation (Placement(transformation(extent={{-140,40},{-100,80}})));

      annotation (
      defaultComponentModel="iceTan",
      Documentation(info="<html>
<p>
This model implements an ice tank model with built-in idealized control
that tracks the set point <code>TSet</code> for the temperature of the working fluid
that leaves the tank, as shown in the figure below.
</p>
<p align=\"center\">
<img alt=\"Schematics of the controlled tank\"
src=\"modelica://Buildings/Resources/Images/Fluid/Storage/Ice/ControlledTank.png\"/>
</p>
<p>
The model is identical to
<a href=\"modelica://Buildings.Fluid.Storage.Ice.Tank\">Buildings.Fluid.Storage.Ice.Tank</a>,
except that it takes as an input the set point for the temperature of the
leaving working fluid.
This temperature is maintained if the flow rate and temperatures allow
sufficient heat flow rate between the tank and the working fluid.
</p>
<p>
The built-in control is an idealization of a tank that has a controller that
bypasses some of the working fluid in order to meet the set point for the temperature
of the leaving working fluid.
The fluid from <code>port_a</code> to <code>port_b</code> has by default
a first order response. If the tank has sufficient capacity for the given
inlet temperature and flow rate, then the idealized control has no
steady-state error. During transients, the set point may not be met
exactly due to the first order response that approximates the dynamics
of the heat exchanger.
</p>
<p>
Note that the setpoint is also tracked during charging mode.
If the full flow rate should go through the tank during charging,
which is generally desired, then set <code>TSet</code> to a
high temperature, such as <i>20</i>&deg;C.
</p>
<h4>Usage</h4>
<p>
This model requires the fluid to flow from <code>port_a</code> to <code>port_b</code>.
Otherwise, the simulation stops with an error.
</p>
</html>",       revisions="<html>
<ul>
<li>
January 26, 2022, by Michael Wetter:<br/>
Refactored model to new architecture.
Changed model to allow idealized control.
Avoided SOC to be outside <i>[0, 1]</i>.
</li>
<li>
December 14, 2021, by Yangyang Fu:<br/>
First implementation.
</li>
</ul>
</html>"),Icon(graphics={
          Text( extent={{-168,130},{-72,84}},
                textColor={0,0,88},
                textString=DynamicSelect("TSet", String(TSet-273.15, format=".1f")))}));
      end ControlledTank;

      model Tank "Ice tank with performance based on performance curves"
        extends
          ProsNet.Fluid.Building_Fluid.Interfaces.TwoPortHeatMassExchanger(
          final allowFlowReversal=false,
          final tau=tauHex,
          final energyDynamics=energyDynamicsHex,
          redeclare final
            ProsNet.Fluid.Building_Fluid.MixingVolumes.MixingVolume vol);

        parameter Real SOC_start(min=0, max=1, final unit="1")
         "Start value for state of charge"
          annotation(Dialog(tab = "Initialization"));

        replaceable parameter
          ProsNet.Under_Development.Storage.Storage.Ice.Data.Tank.Generic per
          "Performance data" annotation (choicesAllMatching=true, Placement(
              transformation(extent={{40,60},{60,80}}, rotation=0)));

        parameter Modelica.Fluid.Types.Dynamics energyDynamicsHex=
          Modelica.Fluid.Types.Dynamics.DynamicFreeInitial
          "Formulation of energy balance for heat exchanger internal fluid mass"
          annotation(Evaluate=true, Dialog(tab = "Dynamics heat exchanger", group="Conservation equations"));

        parameter Modelica.Units.SI.Time tauHex(min=1) = 30
          "Time constant of working fluid through the heat exchanger at nominal flow"
           annotation (Dialog(tab = "Dynamics heat exchanger", group="Conservation equations"));

        // Initialization
        parameter Medium.AbsolutePressure p_start = Medium.p_default
          "Start value of pressure"
          annotation(Dialog(tab = "Initialization"));
        parameter Medium.Temperature T_start = Medium.T_default
          "Start value of temperature"
          annotation(Dialog(tab = "Initialization"));
        parameter Medium.MassFraction X_start[Medium.nX](
          final quantity=Medium.substanceNames) = Medium.X_default
          "Start value of mass fractions m_i/m"
          annotation (Dialog(tab="Initialization", enable=Medium.nXi > 0));
        parameter Medium.ExtraProperty C_start[Medium.nC](
          final quantity=Medium.extraPropertiesNames)=fill(0, Medium.nC)
          "Start value of trace substances"
          annotation (Dialog(tab="Initialization", enable=Medium.nC > 0));

        final parameter Modelica.Units.SI.SpecificHeatCapacity cp=
          Medium.specificHeatCapacityCp(
            Medium.setState_pTX(
              p=Medium.p_default,
              T=273.15,
              X=Medium.X_default))
          "Specific heat capacity of working fluid";
        Modelica.Blocks.Interfaces.RealOutput SOC(
          final unit = "1")
          "state of charge"
          annotation (Placement(transformation(extent={{100,-50},{120,-30}}),
              iconTransformation(extent={{100,-50},{120,-30}})));

        Modelica.Blocks.Interfaces.RealOutput T(
          final quantity="ThermodynamicTemperature",
          final unit = "K",
          displayUnit = "degC",
          min=0)
          "Temperature of the fluid leaving at port_b"
          annotation (Placement(transformation(extent={{100,70},{120,90}})));

        Modelica.Blocks.Interfaces.RealOutput mIce(
          quantity="Mass",
          unit="kg") "Mass of remaining ice"
          annotation (Placement(transformation(extent={{100,-90},{120,-70}}),
              iconTransformation(extent={{100,-90},{120,-70}})));

        Modelica.Blocks.Interfaces.RealOutput Q_flow(final unit="W")
          "Heat flow rate, positive during charging, negative when melting the ice"
          annotation (Placement(transformation(extent={{100,30},{120,50}}),
              iconTransformation(extent={{100,30},{120,50}})));

      protected
        ProsNet.Under_Development.Storage.Storage.Ice.BaseClasses.Tank tanHeaTra(
          final SOC_start=SOC_start,
          final per=per,
          final cp=cp)
          "Model for tank heat transfer between working fluid and ice"
          annotation (Placement(transformation(extent={{-40,-80},{-20,-60}})));

        Buildings.HeatTransfer.Sources.PrescribedHeatFlow preHeaFlo
          "Prescribed heat flow"
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=180,
              origin={-11,-40})));
        Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temSen
          "Temperature of fluid"
          annotation (Placement(transformation(extent={{10,-10},{-10,10}},
              rotation=0,
              origin={-40,-40})));

        Modelica.Blocks.Sources.RealExpression TIn(
          final y=Medium.temperature(state=
              Medium.setState_phX(
              p=port_a.p,
              h=inStream(port_a.h_outflow),
              X=inStream(port_a.Xi_outflow)))) "Inlet temperature into tank"
          annotation (Placement(transformation(extent={{-90,-74},{-70,-54}})));

        Modelica.Blocks.Sources.RealExpression limQ_flow(y=m_flow*cp*(per.TFre - TIn.y))
         "Upper/Lower limit for charging/discharging rate"
          annotation (Placement(transformation(extent={{-90,-90},{-70,-70}})));
      equation
        connect(tanHeaTra.TIn, TIn.y) annotation (Line(points={{-42,-64},{-69,-64}},
                                 color={0,0,127}));
        connect(preHeaFlo.port, vol.heatPort) annotation (Line(points={{-21,-40},{-26,
                -40},{-26,-10},{-9,-10}},                color={191,0,0}));
        connect(tanHeaTra.SOC, SOC) annotation (Line(points={{-19,-70},{80,-70},{80,-40},
                {110,-40}}, color={0,0,127}));
        connect(tanHeaTra.mIce, mIce) annotation (Line(points={{-19,-74},{80,-74},{80,
                -80},{110,-80}}, color={0,0,127}));
        connect(limQ_flow.y, tanHeaTra.QLim_flow) annotation (Line(points={{-69,-80},
                {-56,-80},{-56,-76},{-42,-76}},color={0,0,127}));
        connect(tanHeaTra.Q_flow, Q_flow) annotation (Line(points={{-19,-66},{10,-66},
                {10,-40},{74,-40},{74,40},{110,40}},    color={0,0,127}));
        connect(tanHeaTra.Q_flow, preHeaFlo.Q_flow) annotation (Line(points={{-19,-66},
                {10,-66},{10,-40},{-1,-40}},color={0,0,127}));
        connect(temSen.T, T) annotation (Line(points={{-51,-40},{-80,-40},{-80,50},{
                74,50},{74,80},{110,80}},
              color={0,0,127}));
        connect(temSen.T, tanHeaTra.TOut) annotation (Line(points={{-51,-40},{-54,-40},
                {-54,-70},{-42,-70}}, color={0,0,127}));
        connect(vol.heatPort, temSen.port) annotation (Line(points={{-9,-10},{-26,-10},
                {-26,-40},{-30,-40}}, color={191,0,0}));
        annotation (defaultComponentModel="iceTan", Icon(graphics={
              Rectangle(
                extent={{-70,60},{70,-60}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={127,0,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-52,52},{-36,-54}},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None,
                lineColor={0,0,0}),
              Rectangle(
                extent={{-24,52},{-8,-54}},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None,
                lineColor={0,0,0}),
              Rectangle(
                extent={{4,52},{20,-54}},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None,
                lineColor={0,0,0}),
              Rectangle(
                extent={{32,52},{48,-54}},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None,
                lineColor={0,0,0}),
              Rectangle(
                extent={{-92,6},{92,-4}},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None,
                lineColor={0,0,0}),
              Text(
                extent={{100,72},{140,46}},
                textColor={0,0,88},
                textString="Q_flow"),
              Text(
                extent={{100,-48},{128,-72}},
                textColor={0,0,88},
                textString="mIce"),
              Rectangle(
                extent=DynamicSelect({{70,-60},{84,60}},{{85,-60},{70,-60+(SOC)*120}}),
                fillColor={175,175,175},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None,
                lineColor={0,0,0}),
              Text(
                extent={{100,-10},{128,-34}},
                textColor={0,0,88},
                textString="SOC"),
              Text(
                extent={{90,110},{124,88}},
                textColor={0,0,88},
                textString="T")}),
          Documentation(info="<html>
<p>
This model implements an ice tank model whose performance is computed based on
performance curves.
</p>
<p>
The model is based on the implementation of
<a href=\"https://doi.org/10.3384/ecp21181177\">Guowen et al., 2020</a> and
similar to the detailed EnergyPlus ice tank model
<a href=\"https://bigladdersoftware.com/epx/docs/9-0/input-output-reference/group-plant-equipment.html#thermalstorageicedetailed\">ThermalStorage:Ice:Detailed</a>.
</p>
<p>
The governing equations are as follows:
</p>
<p>
The mass of ice in the storage <i>m<sub>ice</sub></i> is calculated as
</p>
<p align=\"center\" style=\"font-style:italic;\">
d SOC/dt = Q&#775;/(H<sub>f</sub> &nbsp; m<sub>ice,max</sub>)
</p>
<p align=\"center\" style=\"font-style:italic;\">
m<sub>ice</sub> = SOC &nbsp; m<sub>ice,max</sub>
</p>
<p>
where <i>SOC</i> is state of charge,
<i>Q&#775;</i> is the heat transfer rate of the ice tank, positive for charging and negative for discharging,
<i>Hf</i> is the fusion of heat of ice and
<i>m<sub>ice,max</sub></i> is the nominal mass of ice in the storage tank.
</p>
<p>
The heat transfer rate of the ice tank <i>Q&#775;</i> is computed using
</p>
<p align=\"center\" style=\"font-style:italic;\">
Q&#775; = Q<sub>sto,nom</sub> &nbsp; q<sup>*</sup>,
</p>
<p>
where
<i>Q<sub>sto,nom</sub></i> is the storage capacity and
<i>q<sup>*</sup></i> is a normalized heat flow rate.
The storage capacity is
<p align=\"center\" style=\"font-style:italic;\">
Q<sub>sto,nom</sub> = Hf &nbsp; m<sub>ice,max</sub>,
</p>
<p>
where <i>Hf</i> is the latent heat of fusion of ice and
<i>m<sub>ice,max</sub></i> is the maximum ice storage capacity.
</p>
<p>
The normalized heat flow rate is computed using performance curves
for charging (freezing) or discharging (melting).
For charging, the heat transfer rate <i>q*</i> between the chilled water
and the ice in the thermal storage tank is calculated using
</p>
<p align=\"center\">
<i>
q<sup>*</sup> &Delta;t = C<sub>1</sub> + C<sub>2</sub>x + C<sub>3</sub> x<sup>2</sup> + [C<sub>4</sub> + C<sub>5</sub>x + C<sub>6</sub> x<sup>2</sup>]&Delta;T<sub>lmtd</sub><sup>*</sup>
</i>
</p>
<p>where <i>&Delta;t</i> is the time step of the data samples used for the curve fitting,
<i>C<sub>1-6</sub></i> are the curve fit coefficients,
<i>x</i> is the fraction of charging, also known as the state-of-charge,
and <i>T<sub>lmtd</sub><sup>*</sup></i> is the normalized LMTD
calculated using <a href=\"mdoelica://Buildings.Fluid.Storage.Ice.BaseClasses.calculateLMTDStar\">
Buildings.Fluid.Storage.Ice.BaseClasses.calculateLMTDStar</a>.
Similarly, for discharging, the heat transfer rate <i>q*</i>
between the chilled water and the ice in the thermal storage tank is
</p>
<p align=\"center\" style=\"font-style:italic;\">
- q<sup>*</sup> &Delta;t = D<sub>1</sub> + D<sub>2</sub>(1-x) + D<sub>3</sub> (1-x)<sup>2</sup> + [D<sub>4</sub> + D<sub>5</sub>(1-x) + D<sub>6</sub> (1-x)<sup>2</sup>]&Delta;T<sub>lmtd</sub><sup>*</sup>
</p>
<p>
where <i>&Delta;t</i> is the time step of the data samples used for the curve fitting,
<i>D<sub>1-6</sub></i> are the curve fit coefficients.
<p>
The normalized LMTD <i>&Delta;T<sub>lmtd</sub><sup>*</sup></i> uses a nominal temperature difference of 10 Kelvin.
This value must be used when obtaining the curve fit coefficients.
</p>
<p>
The log mean temperature difference is calculated using
</p>
<p align=\"center\" style=\"font-style:italic;\">
    &Delta;T<sub>lmtd</sub><sup>*</sup> = &Delta;T<sub>lmtd</sub>/T<sub>nom</sub>
</p>
<p align=\"center\" style=\"font-style:italic;\">
 &Delta;T<sub>lmtd</sub> = (T<sub>in</sub> - T<sub>out</sub>)/ln((T<sub>in</sub> - T<sub>fre</sub>)/(T<sub>out</sub> - T<sub>fre</sub>))
</p>
<p>
where <i>T<sub>in</sub></i> is the inlet temperature, <i>T<sub>out</sub></i> is the outlet temperature,
<i>T<sub>fre</sub></i> is the freezing temperature
and <i>T<sub>nom</sub></i> is a nominal temperature difference of 10 Kelvin.
</p>
<h4>Usage</h4>
<p>
This model requires the fluid to flow from <code>port_a</code> to <code>port_b</code>.
Otherwise, the simulation stops with an error.
</p>
<h4>
Reference
</h4>
<p>
Strand, R.K. 1992. “Indirect Ice Storage System Simulation,” M.S. Thesis,
Department of Mechanical and Industrial Engineering, University of Illinois at Urbana-Champaign.
</p>
<p>
Guowen Li, Yangyang Fu, Amanda Pertzborn, Jin Wen and Zheng O'Neill.
<i>An Ice Storage Tank Modelica Model: Implementation and Validation.</i> Modelica Conferences. 2021.
<a href=\"https://doi.org/10.3384/ecp21181177\">doi:10.3384/ecp21181177</a>.
</p>
</html>",       revisions="<html>
<ul>
<li>
January 26, 2022, by Michael Wetter:<br/>
Refactored model to new architecture.
Changed model to allow idealized control.
Avoided SOC to be outside <i>[0, 1]</i>.
</li>
<li>
December 14, 2021, by Yangyang Fu:<br/>
First implementation.
</li>
</ul>
</html>"));
      end Tank;

      package Data "Performance data for ice thermal tank"
        extends Modelica.Icons.MaterialPropertiesPackage;

        package Tank "Performance data for ice tank model"
          extends Modelica.Icons.MaterialPropertiesPackage;

          record EnergyPlus =
            ProsNet.Under_Development.Storage.Storage.Ice.Data.Tank.Generic (
              coeCha={0.0,0.09,-0.15,0.612,-0.324,-0.216},
              dtCha=3600,
              coeDisCha={0.0,0.09,-0.15,0.612,-0.324,-0.216},
              dtDisCha=3600)   "Performance curve obtained from EnergyPlus example"
             annotation (defaultComponentName="per",
              Documentation(info="<html>
<p>
The performance curves are obtained from the EnergyPlus example idf file: 
<a href = \"https://github.com/NREL/EnergyPlus/blob/0474bcff0f09143605459c4168e69a17d49e7dd1/testfiles/5ZoneDetailedIceStorage.idf#L3137\">5ZoneDetailedIceStorage.idf</a>.
</p>

</html>"));
          record Experiment =
            ProsNet.Under_Development.Storage.Storage.Ice.Data.Tank.Generic (
              mIce_max=2846.35,
              coeCha={1.76953858E-04,0,0,0,0,0},
              dtCha=10,
              coeDisCha={5.54E-05,-1.45679E-04,9.28E-05,1.126122E-03,-1.1012E-03,
                  3.00544E-04},
              dtDisCha=10)   "Performance curve obtained from onsite experiment"
             annotation (defaultComponentName="per",
              Documentation(info="<html>
<p>
The performance curves are obtained from experiments demonstrated in the following reference.
</p>
<h4>Reference</h4>
<p>
Pradhan, Ojas, et.al. \"Development and Validation of a Simulation Testbed for the Intelligent Building Agents Laboratory (IBAL) using TRNSYS.\" 
ASHRAE Transactions 126 (2020): 458-466.
</p>
<p>
Li, Guowen, et al. \"An Ice Storage Tank Modelica Model: Implementation and Validation.\" Modelica Conferences. 2021.
</p>
</html>"));
          record Generic
            extends Modelica.Icons.Record;
            constant Modelica.Units.SI.TemperatureDifference dT_nominal = 10
               "Nominal temperature difference used for performance data";
            constant Integer nCha = 6 "Number of coefficients for charging characteristic curve";
            constant Integer nDisCha = 6 "Number of coefficients for discharging characteristic curve";

            parameter Modelica.Units.SI.SpecificEnergy Hf = 333550 "Fusion of heat of ice";
            parameter Modelica.Units.SI.Temperature TFre = 273.15
              "Freezing temperature of water or the latent energy storage material";

            parameter Modelica.Units.SI.Mass mIce_max "Maximum mass of ice in the tank";

            parameter Real coeCha[nCha] "Coefficients for charging curve";
            parameter Real coeDisCha[nDisCha] "Coeffcients for discharging curve";
            parameter Real dtCha "Time step of curve fitting data";
            parameter Real dtDisCha "Time step of curve fitting data";

            annotation (defaultComponentName="datIceTan",
               defaultComponentPrefixes="parameter",
              Documentation(revisions="<html>
<ul>
<li>
December 14, 2021, by Yangyang Fu:<br/>
First implementation.
</li>
</ul>
</html>",           info="<html>
<p>
Performance data for ice tank charging and discharging curves.
See
<a href=\"modelica://Buildings.Fluid.Storage.Ice.Tank\">Buildings.Fluid.Storage.Ice.Tank</a>
for the definition of the charging and discharging coefficients.
</p>
</html>"));
          end Generic;
        annotation (Documentation(info="<html>
<p>
This package contains data records that characterize the performance of ice storage tanks.
See
<a href=\"modelica://Buildings.Fluid.Storage.Ice.Tank\">Buildings.Fluid.Storage.Ice.Tank</a>
for the definition of the charging and discharging coefficients.
</p>
</html>"));
        end Tank;
        annotation (Documentation(info="<html>
<p>
Package with performance data for ice storage tanks.
</p>
</html>"));
      end Data;

      package Validation "Package that validates the ice tank model"
        extends Modelica.Icons.ExamplesPackage;

        model Tank "Example that test the tank model"
          extends Modelica.Icons.Example;

          package Medium = Buildings.Media.Antifreeze.PropyleneGlycolWater (
            property_T=293.15,
            X_a=0.30) "Fluid medium";

          parameter Modelica.Units.SI.Mass SOC_start=3/4
            "Start value of ice mass in the tank";
          parameter Modelica.Units.SI.MassFlowRate m_flow_nominal=1
            "Nominal mass flow rate";
          parameter Modelica.Units.SI.PressureDifference dp_nominal=100000
            "Pressure difference";
          parameter
            ProsNet.Under_Development.Storage.Storage.Ice.Data.Tank.Generic per(
            mIce_max=1/4*2846.35,
            coeCha={1.76953858E-04,0,0,0,0,0},
            dtCha=10,
            coeDisCha={5.54E-05,-1.45679E-04,9.28E-05,1.126122E-03,-1.1012E-03,
                3.00544E-04},
            dtDisCha=10) "Tank performance data"
            annotation (Placement(transformation(extent={{60,60},{80,80}})));

          ProsNet.Under_Development.Storage.Storage.Ice.ControlledTank iceTan(
            redeclare package Medium = Medium,
            m_flow_nominal=m_flow_nominal,
            dp_nominal=dp_nominal,
            SOC_start=SOC_start,
            per=per,
            energyDynamicsHex=Modelica.Fluid.Types.Dynamics.FixedInitial)
            annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
          ProsNet.Fluid.Building_Fluid.Sources.MassFlowSource_T sou(
            redeclare package Medium = Medium,
            m_flow=2*m_flow_nominal,
            use_T_in=true,
            nPorts=2)
            annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
          ProsNet.Fluid.Building_Fluid.Sources.Boundary_pT bou(redeclare
              package Medium = Medium, nPorts=2)
            annotation (Placement(transformation(extent={{86,-10},{66,10}})));
          ProsNet.Fluid.Building_Fluid.FixedResistances.PressureDrop res(
            redeclare package Medium = Medium,
            m_flow_nominal=m_flow_nominal,
            dp_nominal=500) "Flow resistance"
            annotation (Placement(transformation(extent={{30,-10},{50,10}})));

          Modelica.Blocks.Sources.CombiTimeTable TSou(
            table=[
              0,       273.15 - 5;
              3600*10, 273.15 - 5;
              3600*10, 273.15 + 10;
              3600*11, 273.15 + 10;
              3600*18, 273.15 + 10;
              3600*18, 273.15 - 5],
              y(each unit="K",
                each displayUnit="degC"))
              "Source temperature"
            annotation (Placement(transformation(extent={{-92,-6},{-72,14}})));

          Modelica.Blocks.Sources.TimeTable TSet(table=[
            0,273.15 -10;
            3600*10,273.15 + 8;
            3600*11,273.15 + 6;
            3600*20,273.15 -12;
            3600*24,273.15 +10], y(unit="K", displayUnit="degC"))
            "Table with set points for leaving water temperature which will be tracked subject to thermodynamic constraints"
            annotation (Placement(transformation(extent={{-92,30},{-72,50}})));
          ProsNet.Under_Development.Storage.Storage.Ice.Tank iceTanUnc(
            redeclare package Medium = Medium,
            m_flow_nominal=m_flow_nominal,
            dp_nominal=dp_nominal,
            SOC_start=SOC_start,
            per=per,
            energyDynamicsHex=Modelica.Fluid.Types.Dynamics.FixedInitial)
            "Uncontrolled ice tank"
            annotation (Placement(transformation(extent={{-10,-50},{10,-30}})));
          Fluid.Building_Fluid.FixedResistances.PressureDrop resUnc(
            redeclare package Medium = Medium,
            m_flow_nominal=m_flow_nominal,
            dp_nominal=500) "Flow resistance"
            annotation (Placement(transformation(extent={{30,-50},{50,-30}})));
        equation
          connect(res.port_b, bou.ports[1])
            annotation (Line(points={{50,0},{62,0},{62,-1},{66,-1}},
                                                     color={0,127,255}));
          connect(TSou.y[1], sou.T_in) annotation (Line(points={{-71,4},{-62,4}},
                            color={0,0,127}));
          connect(TSet.y, iceTan.TSet) annotation (Line(points={{-71,40},{-28,40},{-28,
                  6},{-12,6}}, color={0,0,127}));
          connect(resUnc.port_b, bou.ports[2]) annotation (Line(points={{50,-40},{60,
                  -40},{60,-2},{66,-2},{66,1}}, color={0,127,255}));
          connect(iceTan.port_b, res.port_a)
            annotation (Line(points={{10,0},{30,0}}, color={0,127,255}));
          connect(iceTanUnc.port_b, resUnc.port_a)
            annotation (Line(points={{10,-40},{30,-40}}, color={0,127,255}));
          connect(iceTanUnc.port_a, sou.ports[1]) annotation (Line(points={{-10,-40},{
                  -28,-40},{-28,-1},{-40,-1}}, color={0,127,255}));
          connect(iceTan.port_a, sou.ports[2]) annotation (Line(points={{-10,0},{-26,0},
                  {-26,2},{-40,2}}, color={0,127,255}));
          annotation (
            experiment(
              StartTime=0,
              StopTime=86400,
              Tolerance=1e-06),
            __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Storage/Ice/Validation/Tank.mos"
                "Simulate and Plot"),
            Documentation(info="<html>
<p>
This example is to verify the ice tank model <a href=\"modelica://Buildings.Fluid.Storage.Ice\">Buildings.Fluid.Storage.Ice</a>.
</p>
</html>",         revisions="<html>
<ul>
<li>
December 14, 2021, by Yangyang Fu:<br/>
First implementation.
</li>
</ul>
</html>"));
        end Tank;

        package ExperimentNIST "Validation from experimental data"
          extends Modelica.Icons.ExamplesPackage;

          model Charging "Validation against charging experiment"
            extends
              ProsNet.Under_Development.Storage.Storage.Ice.Validation.ExperimentNIST.BaseClasses.PartialChargingDischarging(
                fileName=Modelica.Utilities.Files.loadResource(
                  "modelica://Buildings/Resources/Data/Fluid/Storage/Ice/Validation/Experiment/charging.txt"),
                SOC_start=0.158);

            annotation (
              experiment(
                StartTime=15000,
                StopTime=60370,
                Tolerance=1e-06),
              __Dymola_Commands(file=
                    "modelica://Buildings/Resources/Scripts/Dymola/Fluid/Storage/Ice/Validation/ExperimentNIST/Charging.mos"
                  "Simulate and Plot"),
              Documentation(info="<html>
<p>
This example is to validate the developed ice tank model for charging mode.
</p>
</html>",           revisions="<html>
<ul>
<li>
December 14, 2021, by Yangyang Fu:<br/>
First implementation.
</li>
</ul>
</html>"));
          end Charging;

          model Discharging1 "Validation against discharging experiment 1"
            extends
              ProsNet.Under_Development.Storage.Storage.Ice.Validation.ExperimentNIST.BaseClasses.PartialChargingDischarging(
              fileName=Modelica.Utilities.Files.loadResource(
                  "modelica://Buildings/Resources/Data/Fluid/Storage/Ice/Validation/Experiment/discharging1.txt"),
              SOC_start=0.90996030,
              offSet(k=-2));

           annotation (
              experiment(
                StartTime = 0,
                StopTime=19990,
                Tolerance=1e-06),
              __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Storage/Ice/Validation/ExperimentNIST/Discharging1.mos"
                  "Simulate and Plot"),
              Documentation(info="<html>
<p>
This example is to validate the developed ice tank model for discharging mode using data generated from experiment 1.
The outlet temperature setpoint is set to 2 degree below the measured temperature because the measurement data was taken when the bypass was fully closed.
In this way, the implemented model will automatically close the bypass valve during charging.
</p>

</html>",           revisions="<html>
<ul>
<li>
December 14, 2021, by Yangyang Fu:<br/>
First implementation.
</li>
</ul>
</html>"));
          end Discharging1;

          model Discharging2 "Validation against discharging experiment 2"
            extends
              ProsNet.Under_Development.Storage.Storage.Ice.Validation.ExperimentNIST.BaseClasses.PartialChargingDischarging(
              fileName=Modelica.Utilities.Files.loadResource(
                  "modelica://Buildings/Resources/Data/Fluid/Storage/Ice/Validation/Experiment/discharging2.txt"),
              SOC_start=0.96645368,
              offSet(k=-2));

            annotation (
              experiment(
                StartTime=0,
                StopTime=36890,
                Tolerance=1e-06),
              __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Storage/Ice/Validation/ExperimentNIST/Discharging2.mos"
                  "Simulate and Plot"),
              Documentation(info="<html>
<p>
This example is to validate the developed ice tank model for discharging mode using data generated from experiment 2.
The outlet temperature setpoint is set to 2 degree below the measured temperature because the measurement data was taken when the bypass was fully closed.
In this way, the implemented model will automatically close the bypass valve during charging.
</p>
</html>",           revisions="<html>
<ul>
<li>
December 14, 2021, by Yangyang Fu:<br/>
First implementation.
</li>
</ul>
</html>"));
          end Discharging2;

          model Discharging3 "Validation against discharging experiment 3"
            extends
              ProsNet.Under_Development.Storage.Storage.Ice.Validation.ExperimentNIST.BaseClasses.PartialChargingDischarging(
              fileName=Modelica.Utilities.Files.loadResource(
                  "modelica://Buildings/Resources/Data/Fluid/Storage/Ice/Validation/Experiment/discharging3.txt"),
              SOC_start=0.969633826,
              offSet(k=-2));

            annotation (
              experiment(
                StartTime=0,
                StopTime=19950,
                Tolerance=1e-06),
              __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Storage/Ice/Validation/ExperimentNIST/Discharging3.mos"
                  "Simulate and Plot"),
              Documentation(info="<html>
<p>
This example is to validate the developed ice tank model for discharging mode using data generated from experiment 3.
The outlet temperature setpoint is set to 2 degree below the measured temperature because the measurement data was taken when the bypass was fully closed.
In this way, the implemented model will automatically close the bypass valve during charging.
</p>
</html>",           revisions="<html>
<ul>
<li>
December 14, 2021, by Yangyang Fu:<br/>
First implementation.
</li>
</ul>
</html>"));
          end Discharging3;

          package BaseClasses "Base classes for validation package"
            extends Modelica.Icons.BasesPackage;

            partial model PartialChargingDischarging "Base example"
              extends Modelica.Icons.Example;
              package Medium = Buildings.Media.Antifreeze.PropyleneGlycolWater
                  (
                property_T=293.15,
                X_a=0.30);
              parameter String fileName "Calibration data file";

              parameter Real SOC_start=0.90996030
                "Start value of state of charge";
              parameter Modelica.Units.SI.MassFlowRate m_flow_nominal=1
                "Nominal mass flow rate";
              parameter Modelica.Units.SI.PressureDifference dp_nominal=100000
                "Pressure difference";
              parameter
                ProsNet.Under_Development.Storage.Storage.Ice.Data.Tank.Generic
                per(
                mIce_max=2846.35,
                coeCha={1.76953858E-04,0,0,0,0,0},
                dtCha=10,
                coeDisCha={5.54E-05,-1.45679E-04,9.28E-05,1.126122E-03,-1.1012E-03,
                    3.00544E-04},
                dtDisCha=10)
                "Performance curve obtained from onsite experiment" annotation (
                 Placement(transformation(extent={{60,60},{80,80}})));

              ProsNet.Under_Development.Storage.Storage.Ice.ControlledTank iceTan(
                redeclare package Medium = Medium,
                m_flow_nominal=m_flow_nominal,
                dp_nominal=dp_nominal,
                SOC_start=SOC_start,
                per=per,
                energyDynamicsHex=Modelica.Fluid.Types.Dynamics.FixedInitial)
                "Ice tank" annotation (Placement(transformation(extent={{10,-10},
                        {30,10}})));
              ProsNet.Fluid.Building_Fluid.Sources.MassFlowSource_T sou(
                redeclare package Medium = Medium,
                use_m_flow_in=true,
                use_T_in=true,
                nPorts=1) "Mass flow source" annotation (Placement(
                    transformation(extent={{-36,-10},{-16,10}})));
              ProsNet.Fluid.Building_Fluid.Sources.Boundary_pT bou(redeclare
                  package Medium = Medium, nPorts=1) "Pressure source"
                annotation (Placement(transformation(
                    extent={{10,-10},{-10,10}},
                    rotation=270,
                    origin={80,-50})));
              ProsNet.Fluid.Building_Fluid.FixedResistances.PressureDrop res(
                redeclare package Medium = Medium,
                m_flow_nominal=m_flow_nominal,
                dp_nominal=500) "Flow resistance" annotation (Placement(
                    transformation(
                    extent={{-10,-10},{10,10}},
                    rotation=270,
                    origin={80,-20})));
              Modelica.Blocks.Sources.CombiTimeTable dat(
                tableOnFile=true,
                tableName="tab",
                columns=2:5,
                fileName=fileName)
                "Flowrate measurements"
                annotation (Placement(transformation(extent={{-80,60},{-60,80}})));

              Modelica.Thermal.HeatTransfer.Celsius.ToKelvin TIn
                "Conversion from Celsius to Kelvin"
                annotation (Placement(transformation(extent={{-74,24},{-54,44}})));
              Modelica.Thermal.HeatTransfer.Celsius.ToKelvin TOutSet
                "Outlet temperature in Kelvin"
                annotation (Placement(transformation(extent={{-62,-30},{-42,-10}})));

              Modelica.Blocks.Math.Add TSet "Temperature setpoint"
                annotation (Placement(transformation(extent={{-30,-60},{-10,-40}})));
              Modelica.Blocks.Sources.Constant offSet(k=0) "An offset for setpoint control"
                annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));

              Fluid.Building_Fluid.Sensors.TemperatureTwoPort TOut(
                redeclare package Medium = Medium,
                allowFlowReversal=false,
                m_flow_nominal=m_flow_nominal,
                tau=0) "Tank outlet temperature" annotation (Placement(
                    transformation(extent={{40,-10},{60,10}})));
            equation
              connect(sou.ports[1], iceTan.port_a)
                annotation (Line(points={{-16,0},{10,0}},  color={0,127,255}));
              connect(res.port_b, bou.ports[1])
                annotation (Line(points={{80,-30},{80,-40}},
                                                         color={0,127,255}));
              connect(dat.y[3], sou.m_flow_in) annotation (Line(points={{-59,70},{-46,70},
                      {-46,8},{-38,8}},
                                   color={0,0,127}));
              connect(dat.y[1], TIn.Celsius) annotation (Line(points={{-59,70},{-52,70},{
                      -52,48},{-80,48},{-80,34},{-76,34}},
                                                       color={0,0,127}));
              connect(TIn.Kelvin, sou.T_in) annotation (Line(points={{-53,34},{-48,34},{
                      -48,4},{-38,4}},
                                   color={0,0,127}));
              connect(dat.y[2], TOutSet.Celsius) annotation (Line(points={{-59,70},{-52,70},
                      {-52,48},{-80,48},{-80,-20},{-64,-20}}, color={0,0,127}));
              connect(TOutSet.Kelvin, TSet.u1) annotation (Line(points={{-41,-20},{-36,-20},
                      {-36,-44},{-32,-44}}, color={0,0,127}));
              connect(offSet.y, TSet.u2) annotation (Line(points={{-59,-50},{-42,-50},{
                      -42,-56},{-32,-56}},
                                  color={0,0,127}));
              connect(TSet.y, iceTan.TSet) annotation (Line(points={{-9,-50},{-2,-50},{-2,
                      6},{8,6}}, color={0,0,127}));
              connect(iceTan.port_b, TOut.port_a)
                annotation (Line(points={{30,0},{40,0}}, color={0,127,255}));
              connect(TOut.port_b, res.port_a)
                annotation (Line(points={{60,0},{80,0},{80,-10}}, color={0,127,255}));
              annotation (
                Documentation(info="<html>
<p>
Basic model that is used to validate the tank model.
The performance data record <code>per</code> contains the data
obtained from experiments of Ojas et al., 2020, and used by Guowen et al., 2021.
</p>
<h4>Reference</h4>
<p>
Pradhan, Ojas, et.al. <i>Development and Validation of a Simulation Testbed for the Intelligent Building Agents Laboratory (IBAL) using TRNSYS.</i>
ASHRAE Transactions 126 (2020): 458-466.
</p>
<p>
Li, Guowen, et al. <i>An Ice Storage Tank Modelica Model: Implementation and Validation.</i> Modelica Conferences. 2021.
<a href=\"https://doi.org/10.3384/ecp21181177\">doi:10.3384/ecp21181177</a>.
</p>
</html>",             revisions="<html>
<ul>
<li>
December 14, 2021, by Yangyang Fu:<br/>
First implementation.
</li>
</ul>
</html>"));
            end PartialChargingDischarging;
          annotation (Documentation(info="<html>
<p>
This package contains base classes that are used to construct the models in
<a href=\"modelica://Buildings.Fluid.Storage.Ice.Validation\">Buildings.Fluid.Storage.Ice.Validation</a>.
</p>
</html>"));
          end BaseClasses;
        annotation (Documentation(info="<html>
<p>
This package contains calibration examples using experimental data from
the Intelligent Building Agents Laboraotry at the National Institute of Standards and Technology (NIST).
The  ice storage  tank  at  NIST  contains 3,105 L of  water and when fully frozen the ice has a capacity of 274 kWh,
designed to be discharged over an eight-hour period with an inlet temperature of 10 °C.
The chilled water that flows through the ice tank is a 30 % PG and 70 % water solution,
and the heat exchanger inside the ice tank is a spiral wound polyethylene tube.
The chiller used for charging has a nominal capacity of 36.1 kW.
However, due to the off-design conditions and wornout, the ice tank has a capacity of abour 264 kWh.
<p>

The experimental data are located at \"IceStorage/Resources/data/Experiment\".

The data are prepared into 5 columns, and the description of each column is explained below.
</p>
<ul>
<li>
1st column: time step in seconds
</li>
<li>
2nd column: measured ice tank inlet temperature in Celsius
</li>
<li>
3rd column: measured ice tank outlet temperature in Celsius
</li>
<li>
4th column: measured ice tank fluid mass flow rate in kg/s
</li>
<li>
5th column: measured dimensionless ice tank state-of-charge
</li>
</ul>
<p>
The data are generated with bypass valve fully closed, and collected an interval of 10 seconds.
</p>
<h4>Calibration</h4>
<p>
The performance curves were calibrated using curve-fitting techniques based on the measurement.
For dishcarging scenario, three experiments were conducted and used for calibration.
The discharging capacity of the ice tank depends on the current state-of-charge, the inlet and outlet temperature of the ice tank.
For charging scenario, one set of experiment data was used in this package.
When under charging, the chiller operated at a constant cooling rate, and therefore the ice tank charging rate was almost constant.
An average of the measured charging rate, 167.79 kW, during the experiment was used for the calibration.
</p>

<h4>Reference</h4>
<p>
Li, Guowen, et al. \"An Ice Storage Tank Modelica Model: Implementation and Validation.\" Modelica Conferences. 2021.
</p>

</html>"));
        end ExperimentNIST;
      annotation (Documentation(info="<html>
<p>
This package contains examples that validate the ice tank from different data sources.
</p>
</html>"));
      end Validation;

      package BaseClasses "Base classes for ice tank models"
        extends Modelica.Icons.BasesPackage;

        model LMTDStar
          "Normalized log mean temperature difference across the ice storage unit"
          extends Modelica.Blocks.Icons.Block;

          parameter Modelica.Units.SI.Temperature TFre = 273.15 "Freezing temperature of water or the latent energy storage material";
          parameter Modelica.Units.SI.TemperatureDifference dT_nominal = 10
           "Nominal temperature difference";

          Modelica.Blocks.Interfaces.RealInput TIn(unit="K", displayUnit="degC")
            "Inlet temperature" annotation (Placement(transformation(extent={{-140,20},{
                    -100,60}}), iconTransformation(extent={{-140,20},{-100,60}})));
          Modelica.Blocks.Interfaces.RealInput TOut(unit="K", displayUnit="degC")
            "Outlet temperature" annotation (Placement(transformation(extent={{-140,-60},
                    {-100,-20}}), iconTransformation(extent={{-140,-60},{-100,-20}})));

          Modelica.Blocks.Interfaces.RealOutput lmtdSta(final quantity="1")
            "Normalized LMTD" annotation (Placement(transformation(extent={{100,-10},{
                    120,10}}), iconTransformation(extent={{100,-10},{120,10}})));

        equation

          lmtdSta = Buildings.Utilities.Math.Functions.smoothMin(
                    x1=10,
                    x2=
              ProsNet.Under_Development.Storage.Storage.Ice.BaseClasses.calculateLMTDStar(
                      TIn=TIn,
                      TOut=TOut,
                      TFre=TFre,
                      dT_nominal=dT_nominal),
                    deltaX=1E-6);

          annotation (defaultComponentName = "lmtdSta",
          Icon(coordinateSystem(preserveAspectRatio = false)),                                  Diagram(
                coordinateSystem(preserveAspectRatio=false)),
            Documentation(info="<html>
<p>
This subroutine calculates the log mean temperature difference for the detailed ice storage unit
using the function
<a href=\"modelica://Buildings.Fluid.Storage.Ice.BaseClasses.calculateLMTDStar\">
Buildings.Fluid.Storage.Ice.BaseClasses.calculateLMTDStar</a>.
</p>
</html>",         revisions="<html>
<ul>
<li>
January 26, 2022, by Michael Wetter:<br/>
Refactored model to new architecture.
</li>
<li>
December 8, 2021, by Yangyang Fu:<br/>
First implementation.
</li>
</ul>
</html>"));
        end LMTDStar;

        model NormalizedHeatFlowRate
          "Charging or discharging rate based on the curves"
          extends Modelica.Blocks.Icons.Block;

          parameter Real coeCha[6]
            "Coefficients for charging curve";
          parameter Real dtCha "Time step of curve fitting data";

          parameter Real coeDisCha[6]
            "Coefficients for discharging curve";
          parameter Real dtDisCha "Time step of curve fitting data";

          Modelica.Blocks.Interfaces.RealOutput qNor(final quantity="1/s")
            "Normalized heat transfer rate: charging when postive, discharge when negative"
                                            annotation (Placement(transformation(extent={{100,-10},
                    {120,10}}),           iconTransformation(extent={{100,-10},{120,10}})));
          Modelica.Blocks.Interfaces.RealInput SOC "State of charge"
            annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
          Modelica.Blocks.Interfaces.RealInput lmtdSta "LMTD star"
            annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));

          Modelica.Blocks.Interfaces.BooleanInput canFreeze
            "Set to true if tank can be charged"
            annotation (Placement(transformation(extent={{-140,20},{-100,60}})));
          Modelica.Blocks.Interfaces.BooleanInput canMelt
            "Set to true if tank can be melted"
            annotation (Placement(transformation(extent={{-140,60},{-100,100}})));

        protected
          Modelica.Blocks.Math.Add SOCCom(final k2=-1) "1 - SOC"
            annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
          Modelica.Blocks.Sources.Constant const(k=1) "Constant output of 1"
            annotation (Placement(transformation(extent={{-80,50},{-60,70}})));

          ProsNet.Under_Development.Storage.Storage.Ice.BaseClasses.QStar qStaCha(final
              coeff=coeCha, final dt=dtCha) "q* for charing mode"
            annotation (Placement(transformation(extent={{20,-16},{40,4}})));
          ProsNet.Under_Development.Storage.Storage.Ice.BaseClasses.QStar qStaDisCha(final
              coeff=coeDisCha, final dt=dtDisCha) "q* for discharging mode"
            annotation (Placement(transformation(extent={{20,60},{40,80}})));
          Buildings.Controls.OBC.CDL.Reals.Subtract qSta
            "Effective normalized heat flow rate"
            annotation (Placement(transformation(extent={{60,-10},{80,10}})));

        equation
          connect(qStaCha.lmtdSta, lmtdSta) annotation (Line(points={{18,-12},{0,-12},{
                  0,-60},{-120,-60}},   color={0,0,127}));
          connect(lmtdSta, qStaDisCha.lmtdSta) annotation (Line(points={{-120,-60},{0,
                  -60},{0,64},{18,64}},      color={0,0,127}));
          connect(SOC, qStaCha.x) annotation (Line(points={{-120,0},{-60,0},{-60,-6},{
                  18,-6}}, color={0,0,127}));
          connect(qStaCha.active, canFreeze) annotation (Line(points={{18,0},{-56,0},{
                  -56,40},{-120,40}},  color={255,0,255}));
          connect(qStaDisCha.active, canMelt) annotation (Line(points={{18,76},{-96,76},
                  {-96,80},{-120,80}}, color={255,0,255}));
          connect(qNor, qSta.y)
            annotation (Line(points={{110,0},{82,0}}, color={0,0,127}));
          connect(SOCCom.u2, SOC) annotation (Line(points={{-42,24},{-60,24},{-60,0},{-120,
                  0}}, color={0,0,127}));
          connect(const.y, SOCCom.u1) annotation (Line(points={{-59,60},{-50,60},{-50,36},
                  {-42,36}}, color={0,0,127}));
          connect(qStaDisCha.x, SOCCom.y) annotation (Line(points={{18,70},{-10,70},{
                  -10,30},{-19,30}}, color={0,0,127}));
          connect(qStaCha.qNor, qSta.u1)
            annotation (Line(points={{41,-6},{48,-6},{48,6},{58,6}}, color={0,0,127}));
          connect(qStaDisCha.qNor, qSta.u2) annotation (Line(points={{41,70},{52,70},{52,
                  -6},{58,-6}}, color={0,0,127}));
          annotation (defaultComponentName = "norQSta",
          Icon(coordinateSystem(preserveAspectRatio=false),
            graphics={Text(textColor = {0, 0, 88}, extent = {{-32, 62}, {58, -34}}, textString = "q*")}),
             Diagram(
                coordinateSystem(preserveAspectRatio=false)),
            Documentation(info="<html>
<p>
This blocks calculate the normalized heat transfer rate for the ice tank in charging or discharging mode.
</p>
<p>The module use the following logic:</p>
<ul>
<li>If <code>canFreeze</code> and <code>canMelt</code> are both <code>false</code>: the heat transfer rate is 0</li>
<li>If <code>canMelt = true</code>: the heat transfer rate is the discharging rate calculated
using <a href=\"modelica://Buildings.Fluid.Storage.Ice.BaseClasses.QStar\">Buildings.Fluid.Storage.Ice.BaseClasses.QStar</a>
with coefficients for discharing mode.
</li>
<li>If <code>canFreeze = true</code>: the heat transfer rate is the charging rate calculated
using <a href=\"modelica://Buildings.Fluid.Storage.Ice.BaseClasses.QStar\">Buildings.Fluid.Storage.Ice.BaseClasses.QStar</a>
with coefficients for charging mode.
</li>
</ul>
</html>",         revisions="<html>
<ul>
<li>
January 26, 2022, by Michael Wetter:<br/>
Refactored model to new architecture.
</li>
<li>
December 8, 2021, by Yangyang Fu:<br/>
First implementation.
</li>
</ul>
</html>"));
        end NormalizedHeatFlowRate;

        model QStar "Calculator for q* under charging mode"
          extends Modelica.Blocks.Icons.Block;

          parameter Real coeff[6] "Coefficients for qstar curve";
          parameter Real dt "Time step of curve fitting data";

          Modelica.Blocks.Interfaces.BooleanInput active
            "Set to true if this tank mode can be active"
            annotation (Placement(transformation(extent={{-140,40},{-100,80}})));

          Modelica.Blocks.Interfaces.RealInput x(unit="1")
            "SOC for charging, or 1-SOC for discharging"
            annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));

          Modelica.Blocks.Interfaces.RealInput lmtdSta(unit="1") "Normalized LMTD"
           annotation (Placement(transformation(extent={{-140,-80},{-100,-40}}),
             iconTransformation(extent={{-140,-80},{-100,-40}})));

          Modelica.Blocks.Interfaces.RealOutput qNor(final quantity="1")
            "Normalized heat transfer rate" annotation (Placement(transformation(extent=
                   {{100,-10},{120,10}}), iconTransformation(extent={{100,-10},{120,10}})));

        equation
          if active then
            qNor =Buildings.Utilities.Math.Functions.smoothMax(
              x1=0,
              x2=Buildings.Utilities.Math.Functions.polynomial(x, coeff[1:3]) +
                Buildings.Utilities.Math.Functions.polynomial(x, coeff[4:6])*lmtdSta,
              deltaX=1E-7)/dt;
          else
            qNor = 0;
          end if;

          annotation (defaultComponentName = "qSta",
          Icon(coordinateSystem(preserveAspectRatio = false)),
                Diagram(
                coordinateSystem(preserveAspectRatio=false)),
            Documentation(info="<html>
<p>
This block calculates the normalized heat transfer rate <i>q*</i> between the chilled water
and the ice in the thermal storage tank using
</p>
<p align=\"center\">
<i>
q<sup>*</sup> &Delta;t = C<sub>1</sub> + C<sub>2</sub>x + C<sub>3</sub> x<sup>2</sup> + [C<sub>4</sub> + C<sub>5</sub>x + C<sub>6</sub> x<sup>2</sup>]&Delta;T<sub>lmtd</sub><sup>*</sup>
</i>
</p>
<p>where <i>&Delta;t</i> is the time step of the data samples used for the curve fitting,
<i>C<sub>1-6</sub></i> are the curve fit coefficients,
<i>x</i> is the fraction of charging, also known as the state-of-charge,
and <i>T<sub>lmtd</sub><sup>*</sup></i> is the normalized LMTD
calculated using <a href=\"mdoelica://Buildings.Fluid.Storage.Ice.BaseClasses.calculateLMTDStar\">
Buildings.Fluid.Storage.Ice.BaseClasses.calculateLMTDStar</a>.
</p>
</html>",         revisions="<html>
<ul>
<li>
January 26, 2022, by Michael Wetter:<br/>
Refactored model to new architecture.
</li>
<li>
December 8, 2021, by Yangyang Fu:<br/>
First implementation.
</li>
</ul>
</html>"));
        end QStar;

        model StateOfCharge "Mass of ice remaining in the tank"
          extends Modelica.Blocks.Icons.Block;

          parameter Real SOC_start(min=0, max=1, final unit="1")
           "Start value for state of charge";

          parameter Modelica.Units.SI.Energy E_nominal "Storage capacity";

          Modelica.Blocks.Interfaces.RealInput Q_flow(final unit="W")
            "Heat transfer rate: positive for charging, negative for discharging"
            annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
          Modelica.Blocks.Interfaces.RealOutput QEff_flow(final unit="W")
            "Actual heat flow rate, taking into account 0 &le; SOC &le; 1"
            annotation (Placement(transformation(extent={{100,50},{120,70}}),
                               iconTransformation(extent={{100,50},{120,70}})));
          Modelica.Blocks.Interfaces.RealOutput SOC(
            final min=0,
            final max=1,
            final unit="1")
            "State of charge"
            annotation (Placement(transformation(extent={{100,-10},{120,10}})));
          Boolean charged "True if tank is fully charged";
          Boolean discharged "True if tank is fully discharged";
          Boolean suspend "True if either fully charged or fully discharged";

        initial equation
          SOC = SOC_start;
          charged = SOC_start >= 1;
          discharged = SOC_start <= 0;

        equation
          der(SOC) = if suspend then 0 else Q_flow/E_nominal;
          suspend = charged or discharged;
          QEff_flow = if suspend then 0 else Q_flow;

          when SOC < 0 then
            discharged = true;
            charged = pre(charged);
          elsewhen SOC > 1 then
            charged = true;
            discharged = pre(discharged);
          elsewhen (pre(discharged) and Q_flow > 1) then
            charged = pre(charged);
            discharged = false;
          elsewhen (pre(charged) and Q_flow < -1) then
            charged = false;
            discharged = pre(discharged);
          end when;

          annotation (defaultComponentName="soc",
          Icon(coordinateSystem(preserveAspectRatio=false), graphics={Text(textColor = {0, 0, 88}, extent = {{-42, 48}, {48, -48}}, textString = "SOC")}),
            Diagram(
              coordinateSystem(preserveAspectRatio=false)),
            Documentation(info="<html>
<p>
This block calculates the state of charge using
</p>
<p align=\"center\" style=\"font-style:italic;\">
d SOC/dt = Q&#775;<sub>eff</sub>/(H<sub>f</sub> &nbsp; m<sub>ice,max</sub>)
</p>
<p>
where <i>SOC</i> is the state of charge,
<i>Q&#775;</i> is the heat transfer rate of the ice tank, positive for charging and negative for discharging,
<i>Hf</i> is the fusion of heat of ice and
<i>m<sub>ice,max</sub></i> is the nominal mass of ice in the storage tank.
</p>
<p>
The model sets <i>Q&#775;<sub>eff</sub> = Q&#775;</i>, unless the state of charge is 0 or 1,
in which case <i>Q&#775;<sub>eff</sub></i> is set to zero if it were to lead to
over- or under-charging.
</p>
</html>",         revisions="<html>
<ul>
<li>
January 26, 2022, by Michael Wetter:<br/>
Rewrote model to avoid state of charge to be outside of <i>[0, 1]</i>.
</li>
<li>
December 8, 2021, by Yangyang Fu:<br/>
First implementation.
</li>
</ul>
</html>"));
        end StateOfCharge;

        model Tank "Block to compute the tank heat transfer"
          extends Modelica.Blocks.Icons.Block;

          constant Modelica.Units.SI.TemperatureDifference dTSmall(min=1E-3) = 0.01 "Small temperature difference";

          parameter Real SOC_start(min=0, max=1, final unit="1")
           "Start value for state of charge"
            annotation(Dialog(tab = "Initialization"));

          replaceable parameter
            ProsNet.Under_Development.Storage.Storage.Ice.Data.Tank.Generic per
            "Performance data" annotation (choicesAllMatching=true, Placement(
                transformation(extent={{72,70},{92,90}})));

          final parameter Modelica.Units.SI.Energy E_nominal=per.Hf*per.mIce_max
            "Storage capacity";
          parameter Modelica.Units.SI.SpecificHeatCapacity cp "Specific heat capacity of working fluid";

          Modelica.Blocks.Interfaces.RealInput TIn(final unit="K", displayUnit="degC")
            "Inlet temperature"
            annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
          Modelica.Blocks.Interfaces.RealInput TOut(final unit="K", displayUnit="degC")
            "Outlet temperature"
            annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));

          Modelica.Blocks.Interfaces.RealOutput Q_flow(final unit="W")
            "Actual heat flow rate, taking into account 0 &le; SOC &le; 1"
            annotation (Placement(transformation(extent={{100,30},{120,50}})));

          Modelica.Blocks.Interfaces.RealOutput SOC(
            final unit = "1")
            "state of charge"
            annotation (Placement(transformation(extent={{100,-10},{120,10}}),
                iconTransformation(extent={{100,-10},{120,10}})));
          Modelica.Blocks.Interfaces.RealOutput mIce(
            quantity="Mass",
            unit="kg") "Mass of remaining ice"
            annotation (Placement(transformation(extent={{100,-50},{120,-30}}),
                iconTransformation(extent={{100,-50},{120,-30}})));
          Modelica.Blocks.Interfaces.RealInput QLim_flow(final unit="W")
            "Limit on heat flow rate due to temperatures and mass flow rate"
            annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));

          ProsNet.Under_Development.Storage.Storage.Ice.BaseClasses.StateOfCharge
            SOCCalc(final SOC_start=SOC_start, final E_nominal=E_nominal)
            "State of charge calculation"
            annotation (Placement(transformation(extent={{30,-80},{50,-60}})));

          Buildings.Controls.OBC.CDL.Reals.LessThreshold canFreeze(final t=per.TFre
                 - dTSmall, final h=dTSmall/2)
            "Outputs true if temperatures allow ice to be produced"
            annotation (Placement(transformation(extent={{-50,0},{-30,20}})));
          Buildings.Controls.OBC.CDL.Reals.GreaterThreshold canMelt(final t=per.TFre
                 + dTSmall, final h=dTSmall/2)
            "Outputs true if temperature allows tank to be melted"
            annotation (Placement(transformation(extent={{-50,30},{-30,50}})));

        protected
          Modelica.Blocks.Math.Max QMax_flow "Maximum heat flow rate"
            annotation (Placement(transformation(extent={{-48,-96},{-28,-76}})));
          Modelica.Blocks.Math.Min QMin_flow "Miminum heat flow rate"
            annotation (Placement(transformation(extent={{-48,-66},{-28,-46}})));
          Modelica.Blocks.Logical.Switch QThe_flow
            "Heat flow rate extracted from fluid, not taking into account state of charge"
            annotation (Placement(transformation(extent={{0,-80},{20,-60}})));
          Modelica.Blocks.Math.Gain masIce(
            final k=per.mIce_max,
            y(final unit="kg"))
            "Mass of ice"
            annotation (Placement(transformation(extent={{72,-80},{92,-60}})));
          ProsNet.Under_Development.Storage.Storage.Ice.BaseClasses.NormalizedHeatFlowRate
            norQSta(
            final coeCha=per.coeCha,
            final coeDisCha=per.coeDisCha,
            final dtCha=per.dtCha,
            final dtDisCha=per.dtDisCha)
            annotation (Placement(transformation(extent={{8,66},{28,86}})));
          Modelica.Blocks.Math.Gain QCoe_flow(k=E_nominal, y(final unit="W"))
            "Heat flow rate based on performance curves, without any limitation"
            annotation (Placement(transformation(extent={{36,66},{56,86}})));
          ProsNet.Under_Development.Storage.Storage.Ice.BaseClasses.LMTDStar lmtdSta(final
              TFre=per.TFre, final dT_nominal=per.dT_nominal)
            annotation (Placement(transformation(extent={{-48,60},{-28,80}})));

        equation
          connect(norQSta.qNor, QCoe_flow.u)
            annotation (Line(points={{29,76},{34,76}}, color={0,0,127}));
          connect(QThe_flow.y, SOCCalc.Q_flow)
            annotation (Line(points={{21,-70},{28,-70}}, color={0,0,127}));
          connect(QMin_flow.y, QThe_flow.u1) annotation (Line(points={{-27,-56},{-8,-56},
                  {-8,-62},{-2,-62}}, color={0,0,127}));
          connect(QCoe_flow.y, QMin_flow.u2) annotation (Line(points={{57,76},{60,76},{60,
                  -20},{-56,-20},{-56,-62},{-50,-62}},                     color={0,0,127}));
          connect(QCoe_flow.y, QMax_flow.u1) annotation (Line(points={{57,76},{60,76},{
                  60,-20},{-56,-20},{-56,-80},{-50,-80}},                  color={0,0,127}));
          connect(SOCCalc.SOC, SOC) annotation (Line(points={{51,-70},{64,-70},{64,0},{
                  110,0}},
                       color={0,0,127}));
          connect(QMax_flow.y, QThe_flow.u3) annotation (Line(points={{-27,-86},{-8,-86},
                  {-8,-78},{-2,-78}}, color={0,0,127}));
          connect(lmtdSta.lmtdSta, norQSta.lmtdSta) annotation (Line(points={{-27,70},{6,
                  70}},                          color={0,0,127}));
          connect(SOCCalc.SOC, norQSta.SOC) annotation (Line(points={{51,-70},{64,-70},
                  {64,0},{0,0},{0,76},{6,76}},color={0,0,127}));
          connect(norQSta.canMelt, canMelt.y) annotation (Line(points={{6,84},{-20,84},{
                  -20,40},{-28,40}},
                                   color={255,0,255}));
          connect(canFreeze.y, norQSta.canFreeze) annotation (Line(points={{-28,10},{-12,
                  10},{-12,80},{6,80}},  color={255,0,255}));
          connect(canFreeze.y, QThe_flow.u2) annotation (Line(points={{-28,10},{-12,10},
                  {-12,-70},{-2,-70}}, color={255,0,255}));
          connect(SOCCalc.QEff_flow, Q_flow) annotation (Line(points={{51,-64},{62,-64},
                  {62,40},{110,40}}, color={0,0,127}));
          connect(lmtdSta.TIn, TIn) annotation (Line(points={{-50,74},{-90,74},{-90,60},
                  {-120,60}}, color={0,0,127}));
          connect(canFreeze.u, TIn) annotation (Line(points={{-52,10},{-90,10},{-90,60},
                  {-120,60}}, color={0,0,127}));
          connect(lmtdSta.TOut, TOut) annotation (Line(points={{-50,66},{-80,66},{-80,0},
                  {-120,0}}, color={0,0,127}));
          connect(TIn, canMelt.u) annotation (Line(points={{-120,60},{-90,60},{-90,40},{
                  -52,40}}, color={0,0,127}));
          connect(QLim_flow, QMin_flow.u1) annotation (Line(points={{-120,-60},{-58,-60},
                  {-58,-50},{-50,-50}}, color={0,0,127}));
          connect(QLim_flow, QMax_flow.u2) annotation (Line(points={{-120,-60},{-58,-60},
                  {-58,-92},{-50,-92}}, color={0,0,127}));
          connect(SOCCalc.SOC, masIce.u)
            annotation (Line(points={{51,-70},{70,-70}}, color={0,0,127}));
          connect(masIce.y, mIce) annotation (Line(points={{93,-70},{96,-70},{96,-40},{
                  102,-40}}, color={0,0,127}));
          annotation (Documentation(info="<html>
<p>
Model that implements the heat transfer rate between the working fluid and the ice,
and that computes the state of charge of the tank.
</p>
<p>
See
<a href=\"modelica://Buildings.Fluid.Storage.Ice.Tank\">
Buildings.Fluid.Storage.Ice.Tank</a>
for the implemented equations.
</p>
</html>",         revisions="<html>
<ul>
<li>
January 26, 2022, by Michael Wetter:<br/>
First implementation based on original model, but refactored to simplify model architecture.
</li>
</ul>
</html>"));
        end Tank;

        function calculateLMTDStar
          "This function calculates the log mean temperature difference for the ice storage unit"
          extends Modelica.Icons.Function;
          input Modelica.Units.SI.Temperature TIn "Inlet temperature";
          input Modelica.Units.SI.Temperature TOut "Outlet temperature";

          input Modelica.Units.SI.Temperature TFre = 273.15
            "Freezing temperature of water or the latent energy storage material";
          input Modelica.Units.SI.TemperatureDifference dT_nominal = 10
             "Nominal temperature difference";

          output Real lmtd
            "Normalized LMTD";

        protected
          constant Modelica.Units.SI.TemperatureDifference dTif_min=0.02
            "Small temperature difference, used for regularization";
          constant Modelica.Units.SI.TemperatureDifference dTof_min=0.01
            "Small temperature difference, used for regularization";
          //Modelica.Units.SI.Temperature TOutEps
          //  "Outlet temperature, bounded away from TIn";
          Real dTio "Inlet to outlet temperature difference";
          Real dTif "Inlet to freezing temperature difference";
          Real dTof "Outlet to frezzing temperature difference";
          Real lndT "log of the temperature difference";
          Real eps = 1E-09 "Small tolerance";
        algorithm

          dTio := abs(TIn-TOut);

          dTif := Buildings.Utilities.Math.Functions.smoothMax(
            x1 = abs(TIn-TFre),
            x2 = dTif_min,
            deltaX = eps);
          dTof := Buildings.Utilities.Math.Functions.smoothMax(
            x1 = abs(TOut-TFre),
            x2 = dTof_min,
            deltaX = eps);

        //  lmtd := Buildings.Utilities.Math.Functions.smoothMax(
        //    x1 = 0,
        //    x2 = (dTio/log(dTif/dTof))/dT_nominal,
        //    deltaX = 1E-09);
          lmtd := (dTio/max(eps, log(dTif/dTof)))/dT_nominal;

          annotation (
          smoothOrder=1, Documentation(info="<html>
<p>
This subroutine calculates the log mean temperature difference for the detailed ice storage unit.
The temperature difference is non-dimensionalized using a nominal temperature difference of 10 Kelvin.
This value must be used when obtaining the curve fit coefficients.
</p>
<p>
The log mean temperature difference is calculated using
</p>
<p align=\"center\" style=\"font-style:italic;\">
    T<sub>lmtd</sub><sup>*</sup> = T<sub>lmtd</sub>/T<sub>nom</sub>
</p>
<p align=\"center\" style=\"font-style:italic;\">
 T<sub>lmtd</sub> = (T<sub>in</sub> - T<sub>out</sub>)/ln((T<sub>in</sub> - T<sub>fre</sub>)/(T<sub>out</sub> - T<sub>fre</sub>))
</p>
<p>
where <i>T<sub>in</sub></i> is the inlet temperature, <i>T<sub>out</sub></i> is the outlet temperature,
<i>T<sub>fre</sub></i> is the freezing temperature
and <i>T<sub>nom</sub></i> is a nominal temperature difference of 10 Kelvin.
</p>
</html>",         revisions="<html>
<ul>
<li>
December 8, 2021, by Yangyang Fu:<br/>
First implementation.
</li>
</ul>
</html>"));
        end calculateLMTDStar;

        package Examples "Examples that test models in the base classes"
          extends Modelica.Icons.ExamplesPackage;

          model LMTDStar "Example that tests the LMTDStar model"
            extends Modelica.Icons.Example;

            ProsNet.Under_Development.Storage.Storage.Ice.BaseClasses.LMTDStar lmtdSta
              "LMTD star"
              annotation (Placement(transformation(extent={{32,-10},{52,10}})));
            Modelica.Blocks.Sources.Cosine TIn(
              amplitude=4,
              f=1/3600,
              offset=273.15 + 2) "Inlet temperature"
              annotation (Placement(transformation(extent={{-40,10},{-20,30}})));
            Modelica.Blocks.Sources.Cosine TOut(
              amplitude=4,
              f=1/3600,
              offset=273.15 + 2,
              phase=3.1415926535898) "Outlet temperature"
              annotation (Placement(transformation(extent={{-40,-30},{-20,-10}})));
          equation
            connect(TIn.y, lmtdSta.TIn) annotation (Line(points={{-19,20},{8,20},{8,4},{
                    30,4}},   color={0,0,127}));
            connect(TOut.y, lmtdSta.TOut) annotation (Line(points={{-19,-20},{8,-20},{8,
                    -4},{30,-4}},      color={0,0,127}));
            annotation (
              experiment(StartTime=0,
                        StopTime=3600,
                        Tolerance=1e-06),
              __Dymola_Commands(file=
                    "modelica://Buildings/Resources/Scripts/Dymola/Fluid/Storage/Ice/BaseClasses/Examples/LMTDStar.mos"
                  "Simulate and Plot"),
                  Documentation(info=
               "<html>
      <p>This example is to validate the <code>LMTDStar</code>.</p>
      </html>",
              revisions="<html>
  <ul>
  <li>
  December 8, 2021, by Yangyang Fu:<br/>
  First implementation.
  </li>
  </ul>
</html>"));
          end LMTDStar;

          model NormalizedHeatFlowRate "Example to calculate qStar"
            extends Modelica.Icons.Example;
            parameter Real coeCha[6] = {0, 0.09, -0.15, 0.612, -0.324, -0.216} "Coefficient for charging curve";
            parameter Real coeDisCha[6] = {0, 0.09, -0.15, 0.612, -0.324, -0.216} "Coefficient for discharging curve";
            parameter Real dt = 3600 "Time step used in the samples for curve fitting";

            Modelica.Blocks.Sources.Cosine fra(
              amplitude=0.5,
              offset=0.5,
              f=1/7200) "Fraction of charge"
              annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
            ProsNet.Under_Development.Storage.Storage.Ice.BaseClasses.NormalizedHeatFlowRate
              norQSta(
              coeCha=coeCha,
              dtCha=dt,
              coeDisCha=coeDisCha,
              dtDisCha=dt) "Storage heat transfer rate"
              annotation (Placement(transformation(extent={{60,-10},{80,10}})));

            Modelica.Blocks.Sources.Step lmtd(
              startTime=3600,
              offset=1,
              height=-0.5)
                         "lmtd start"
              annotation (Placement(transformation(extent={{-40,-60},{-20,-40}})));
            Buildings.Controls.OBC.CDL.Reals.GreaterThreshold greThr(t=0.75)
              "Switch the change between charging and discharging mode"
              annotation (Placement(transformation(extent={{-10,30},{10,50}})));
            Buildings.Controls.OBC.CDL.Logical.Not not1
              annotation (Placement(transformation(extent={{20,2},{40,22}})));
          equation
            connect(fra.y, norQSta.SOC)
              annotation (Line(points={{-19,0},{58,0}}, color={0,0,127}));
            connect(lmtd.y, norQSta.lmtdSta) annotation (Line(points={{-19,-50},{-14,-50},
                    {-14,-6},{58,-6}},
                                     color={0,0,127}));
            connect(lmtd.y, greThr.u) annotation (Line(points={{-19,-50},{-14,-50},{-14,
                    40},{-12,40}}, color={0,0,127}));
            connect(norQSta.canMelt, greThr.y) annotation (Line(points={{58,8},{50,8},{50,
                    40},{12,40}}, color={255,0,255}));
            connect(greThr.y, not1.u) annotation (Line(points={{12,40},{16,40},{16,12},{
                    18,12}}, color={255,0,255}));
            connect(not1.y, norQSta.canFreeze) annotation (Line(points={{42,12},{46,12},{
                    46,4},{58,4}}, color={255,0,255}));
            annotation (
              experiment(
                StartTime=0,
                StopTime=7200,
                Tolerance=1e-06),
              __Dymola_Commands(file=
                    "modelica://Buildings/Resources/Scripts/Dymola/Fluid/Storage/Ice/BaseClasses/Examples/NormalizedHeatFlowRate.mos"
                  "Simulate and Plot"),
              Documentation(revisions="<html>
  <ul>
  <li>
  December 8, 2021, by Yangyang Fu:<br/>
  First implementation.
  </li>
  </ul>
  </html>",
          info="<html>
<p>
This example is to validate the
<a href=\"modelica://Buildings.Fluid.Storage.Ice.BaseClasses.NormalizedHeatFlowRate\">
Buildings.Fluid.Storage.Ice.BaseClasses.NormalizedHeatFlowRate</a>.
</p>
</html>"));
          end NormalizedHeatFlowRate;

          model QStar "Example to calculate QStar"
            extends Modelica.Icons.Example;

            parameter Real coeCha[6] = {0, 0.09, -0.15, 0.612, -0.324, -0.216} "Coefficient for charging curve";
            parameter Real dt = 3600 "Time step used in the samples for curve fitting";

            Modelica.Blocks.Sources.Cosine fra(
              amplitude=0.5,
              f=1/86400,
              offset=0.5) "Fraction of charge"
              annotation (Placement(transformation(extent={{-60,-6},{-40,14}})));
            Modelica.Blocks.Sources.Constant lmtd(k=1) "Log mean temperature difference"
              annotation (Placement(transformation(extent={{-60,-40},{-40,-20}})));
            ProsNet.Under_Development.Storage.Storage.Ice.BaseClasses.QStar qSta(coeff=
                  coeCha, dt=dt) annotation (Placement(transformation(extent={{
                      -10,-10},{10,10}})));
            Buildings.Controls.OBC.CDL.Logical.Sources.Constant active(k=true)
              "Outputs true to activate the component" annotation (Placement(
                  transformation(extent={{-60,30},{-40,50}})));
          equation
            connect(fra.y, qSta.x) annotation (Line(points={{-39,4},{-26,4},{-26,0},{-12,
                    0}}, color={0,0,127}));
            connect(lmtd.y, qSta.lmtdSta) annotation (Line(points={{-39,-30},{-26,-30},{
                    -26,-6},{-12,-6}},
                                   color={0,0,127}));
            connect(active.y, qSta.active) annotation (Line(points={{-38,40},{-20,40},{
                    -20,6},{-12,6}}, color={255,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)),
              experiment(StartTime=0,
                        StopTime=86400,
                        Tolerance=1e-06),
              __Dymola_Commands(file=
                    "modelica://Buildings/Resources/Scripts/Dymola/Fluid/Storage/Ice/BaseClasses/Examples/QStar.mos"
                  "Simulate and Plot"),
              Documentation(info="<html>
<p>
This example is to validate the
<a href=\"modelica://Buildings.Fluid.Storage.Ice.BaseClasses.QStar\">
Buildings.Fluid.Storage.Ice.BaseClasses.QStar</a>.
</p>
</html>",           revisions="<html>
  <ul>
  <li>
  December 8, 2021, by Yangyang Fu:<br/>
  First implementation.
  </li>
  </ul>
</html>"));
          end QStar;

          model StateOfCharge "Example that tests the ice mass calculation"
            extends Modelica.Icons.Example;

            ProsNet.Under_Development.Storage.Storage.Ice.BaseClasses.StateOfCharge
              soc(E_nominal=2846.35*333550, SOC_start=1/2) "State of charge"
              annotation (Placement(transformation(extent={{0,-10},{20,10}})));
            Modelica.Blocks.Sources.Cosine q(
              f=1/3600,
              amplitude=1,
              offset=0)
              "Heat transfer rate: postive for charging, negative for discharging"
              annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
            Modelica.Blocks.Math.Gain hf(k=333550) "Fusion of heat of ice"
              annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
          equation
            connect(q.y, hf.u)
              annotation (Line(points={{-59,0},{-42,0}}, color={0,0,127}));
            connect(hf.y, soc.Q_flow)
              annotation (Line(points={{-19,0},{-2,0}}, color={0,0,127}));
            annotation (
              __Dymola_Commands(file=
                    "modelica://Buildings/Resources/Scripts/Dymola/Fluid/Storage/Ice/BaseClasses/Examples/StateOfCharge.mos"
                  "Simulate and Plot"),
              Documentation(info="<html>
<p>This example is to validate the model that calculates the ice mass.</p>
</html>",           revisions="<html>
<ul>
<li>
December 8, 2021, by Yangyang Fu:<br/>
First implementation.
</li>
</ul>
</html>"),    experiment(
                StartTime=0,
                StopTime=3600,
                Tolerance=1e-06));
          end StateOfCharge;
        annotation (Documentation(info="<html>
<p>
This package contains examples that test the models in
<a href=\"modelica://Buildings.Fluid.Storage.Ice.BaseClasses\">Buildings.Fluid.Storage.Ice.BaseClasses</a>.
</p>
</html>"));
        end Examples;
      annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains base classes that are used to construct the models in
<a href=\"modelica://Buildings.Fluid.Storage.Ice\">Buildings.Fluid.Storage.Ice</a>.
</p>
</html>"));
      end BaseClasses;
    annotation (Documentation(info="<html>
<p>
Package with ice thermal storage models.
</p>
</html>"),     Icon(graphics={Line(points={{0,80},{0,-80}},   color={0,128,255},
              thickness=0.5,
              rotation=180),
            Line(
              points={{-40,68},{0,32},{40,68}},
              color={0,128,255},
              thickness=0.5),
            Line(
              points={{-40,-68},{0,-32},{40,-68}},
              color={0,128,255},
              thickness=0.5),
            Line(
              points={{-40,-20},{-1.83697e-15,16},{40,-20}},
              color={0,128,255},
              thickness=0.5,
              origin={48,0},
              rotation=90),
            Line(
              points={{-40,20},{1.83697e-15,-16},{40,20}},
              color={0,128,255},
              thickness=0.5,
              origin={-48,0},
              rotation=90),   Line(points={{0,80},{0,-80}},   color={0,128,255},
              thickness=0.5,
              rotation=270)}));
    end Ice;

    package Examples "Collection of models that illustrate model use and test models"
      extends Modelica.Icons.ExamplesPackage;

      model ExpansionVessel "Test model for expansion vessel"
        extends Modelica.Icons.Example;

      // package Medium = Modelica.Media.Water.WaterIF97OnePhase_ph "Medium model";
       package Medium = Buildings.Media.Water "Medium model";

        ProsNet.Under_Development.Storage.Storage.ExpansionVessel expVes(
            redeclare package Medium = Medium, V_start=1) "Expansion vessel"
          annotation (Placement(transformation(extent={{60,-10},{80,10}})));
        ProsNet.Fluid.Building_Fluid.MixingVolumes.MixingVolume vol(
          redeclare package Medium = Medium,
          V=1,
          nPorts=1,
          m_flow_nominal=0.001,
          energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
          "Volume of water"
          annotation (Placement(transformation(extent={{30,-10},{50,10}})));
        Modelica.Blocks.Sources.Pulse pulse(
          amplitude=20,
          period=3600,
          offset=293.15)
          annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
        Buildings.HeatTransfer.Sources.PrescribedTemperature preTem
          annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
        Modelica.Thermal.HeatTransfer.Components.ThermalConductor theCon(G=10000)
          annotation (Placement(transformation(extent={{0,-10},{20,10}})));
      equation
        connect(pulse.y, preTem.T) annotation (Line(
            points={{-59,0},{-54.75,0},{-54.75,1.27676e-015},{-50.5,1.27676e-015},{-50.5,
                0},{-42,0}},
            color={0,0,127}));
        connect(preTem.port, theCon.port_a) annotation (Line(
            points={{-20,0},{-15,0},{-15,1.22125e-015},{-10,1.22125e-015},{-10,0},{0,0}},
            color={191,0,0}));
        connect(theCon.port_b, vol.heatPort) annotation (Line(
            points={{20,0},{22.5,0},{22.5,1.22125e-015},{25,1.22125e-015},{25,0},{30,0}},
            color={191,0,0}));
        connect(vol.ports[1], expVes.port_a) annotation (Line(
            points={{40,-10},{40,-20},{70,-20},{70,-10}},
            color={0,127,255}));
        annotation (
        __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Storage/Examples/ExpansionVessel.mos"
              "Simulate and plot"),
          Documentation(info="<html>
<p>
This model tests a pressure expansion vessel.
</p>
</html>",       revisions="<html>
<ul>
<li>
December 19, 2022 by Hongxiang Fu:<br/>
Deleted outdated comment in documentation regarding compressible fluid.
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/3198\">#3198</a>.
</li>
</ul>
</html>"),experiment(Tolerance=1e-6, StopTime=7200));
      end ExpansionVessel;

      model Stratified "Test model for stratified tank"
        extends Modelica.Icons.Example;

       package Medium = Buildings.Media.Water "Medium model";

        ProsNet.Under_Development.Storage.Storage.Stratified tanSim(
          redeclare package Medium = Medium,
          hTan=3,
          dIns=0.3,
          nSeg=10,
          m_flow_nominal=0.1,
          VTan=3) "Tank"
          annotation (Placement(transformation(extent={{-20,0},{0,20}})));
          Modelica.Blocks.Sources.TimeTable TWat(table=[0,273.15 + 40; 3600,273.15 +
              40; 3600,273.15 + 20; 7200,273.15 + 20]) "Water temperature"
                       annotation (Placement(transformation(extent={{-100,2},{-80,22}})));
        ProsNet.Fluid.Building_Fluid.Sources.Boundary_pT sou_1(
          p=300000 + 5000,
          T=273.15 + 50,
          redeclare package Medium = Medium,
          use_T_in=true,
          nPorts=2)
          annotation (Placement(transformation(extent={{-60,-2},{-40,18}})));
        ProsNet.Fluid.Building_Fluid.Sources.Boundary_pT sin_1(
          redeclare package Medium = Medium,
          T=273.15 + 20,
          use_p_in=true,
          p=300000,
          nPorts=2)
          annotation (Placement(transformation(extent={{90,-18},{70,2}})));
        ProsNet.Fluid.Building_Fluid.FixedResistances.PressureDrop res_1(
          from_dp=true,
          redeclare package Medium = Medium,
          dp_nominal=5000,
          m_flow_nominal=0.1)
          annotation (Placement(transformation(extent={{34,-18},{54,2}})));
        ProsNet.Under_Development.Storage.Storage.StratifiedEnhanced tanEnh(
          redeclare package Medium = Medium,
          hTan=3,
          dIns=0.3,
          nSeg=10,
          m_flow_nominal=0.1,
          VTan=3) "Tank"
          annotation (Placement(transformation(extent={{-20,-70},{0,-50}})));
        ProsNet.Fluid.Building_Fluid.FixedResistances.PressureDrop res_2(
          from_dp=true,
          redeclare package Medium = Medium,
          dp_nominal=5000,
          m_flow_nominal=0.1)
          annotation (Placement(transformation(extent={{40,-90},{60,-70}})));
        ProsNet.Fluid.Building_Fluid.Sensors.EnthalpyFlowRate HOut_flow(
            redeclare package Medium = Medium, m_flow_nominal=0.1)
          "Enthalpy flow rate"
          annotation (Placement(transformation(extent={{4,-16},{20,0}})));
        ProsNet.Fluid.Building_Fluid.Sensors.EnthalpyFlowRate HOut_flow1(
            redeclare package Medium = Medium, m_flow_nominal=0.1)
          "Enthalpy flow rate"
          annotation (Placement(transformation(extent={{20,-88},{36,-72}})));
        Modelica.Blocks.Continuous.Integrator dH
          "Differenz in enthalpy (should be zero at steady-state)"
          annotation (Placement(transformation(extent={{68,30},{88,50}})));
        Modelica.Blocks.Math.Add add(k2=-1) annotation (Placement(transformation(
                extent={{32,30},{52,50}})));
          Modelica.Blocks.Sources.TimeTable P(table=[0,300000; 4200,300000; 4200,
              305000; 7200,305000; 7200,310000; 10800,310000; 10800,305000])
          "Pressure boundary condition"
                       annotation (Placement(transformation(extent={{20,60},{40,80}})));
        Modelica.Blocks.Sources.Sine sine(
          f=1/86400,
          amplitude=10,
          offset=273.15 + 20)
          annotation (Placement(transformation(extent={{-90,62},{-70,82}})));
        Buildings.HeatTransfer.Sources.PrescribedTemperature TBCSid2
          "Boundary condition for tank" annotation (Placement(transformation(extent={
                  {-40,50},{-28,62}})));
        Buildings.HeatTransfer.Sources.PrescribedTemperature TBCSid1
          "Boundary condition for tank" annotation (Placement(transformation(extent={
                  {-40,84},{-28,96}})));
        Buildings.HeatTransfer.Sources.PrescribedTemperature TBCTop1
          "Boundary condition for tank" annotation (Placement(transformation(extent={
                  {-40,66},{-28,78}})));
        Buildings.HeatTransfer.Sources.PrescribedTemperature TBCTop2
          "Boundary condition for tank" annotation (Placement(transformation(extent={
                  {-40,32},{-28,44}})));
      equation
        connect(TWat.y, sou_1.T_in) annotation (Line(
            points={{-79,12},{-62,12}},
            color={0,0,127}));
        connect(tanSim.port_b, HOut_flow.port_a) annotation (Line(points={{-10,0},{
                -10,-8},{4,-8}},           color={0,127,255}));
        connect(HOut_flow.port_b, res_1.port_a)
          annotation (Line(points={{20,-8},{34,-8}}, color={0,127,255}));
        connect(tanEnh.port_b, HOut_flow1.port_a)
          annotation (Line(points={{-10,-70},{-10,-80},{20,-80}},
                                                     color={0,127,255}));
        connect(HOut_flow1.port_b, res_2.port_a) annotation (Line(points={{36,-80},{
                40,-80}}, color={0,127,255}));
        connect(add.y, dH.u)
          annotation (Line(points={{53,40},{66,40}},   color={0,0,127}));
        connect(HOut_flow.H_flow, add.u1) annotation (Line(points={{12,0.8},{12,46},{
                30,46}},   color={0,0,127}));
        connect(HOut_flow1.H_flow, add.u2) annotation (Line(points={{28,-71.2},{28,34},
                {30,34}},       color={0,0,127}));
        connect(P.y, sin_1.p_in) annotation (Line(
            points={{41,70},{100,70},{100,0},{92,0}},
            color={0,0,127}));
        connect(sine.y, TBCSid1.T) annotation (Line(points={{-69,72},{-55.5,72},{
                -55.5,90},{-41.2,90}}, color={0,0,127}));
        connect(sine.y, TBCTop1.T) annotation (Line(points={{-69,72},{-41.2,72}},
              color={0,0,127}));
        connect(sine.y, TBCSid2.T) annotation (Line(points={{-69,72},{-56,72},{-56,56},
                {-41.2,56}}, color={0,0,127}));
        connect(sine.y, TBCTop2.T) annotation (Line(points={{-69,72},{-56,72},{-56,38},
                {-41.2,38}}, color={0,0,127}));
        connect(TBCSid2.port, tanEnh.heaPorSid) annotation (Line(points={{-28,56},{
                -24,56},{-24,-12},{-4.4,-12},{-4.4,-60}},        color={191,0,0}));
        connect(TBCTop2.port, tanEnh.heaPorTop) annotation (Line(points={{-28,38},{
                -26,38},{-26,-14},{-8,-14},{-8,-52.6}}, color={191,0,0}));
        connect(sin_1.ports[1], res_1.port_b) annotation (Line(
            points={{70,-9},{64,-9},{64,-8},{54,-8}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(sin_1.ports[2], res_2.port_b) annotation (Line(
            points={{70,-7},{64,-7},{64,-80},{60,-80}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(sou_1.ports[1], tanSim.port_a) annotation (Line(
            points={{-40,7},{-30,7},{-30,20},{-10,20}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(sou_1.ports[2], tanEnh.port_a) annotation (Line(
            points={{-40,9},{-30,9},{-30,-40},{-10,-40},{-10,-50}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(TBCSid1.port, tanSim.heaPorSid) annotation (Line(
            points={{-28,90},{-4.4,90},{-4.4,10}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(TBCTop1.port, tanSim.heaPorTop) annotation (Line(
            points={{-28,72},{-8,72},{-8,17.4}},
            color={191,0,0},
            smooth=Smooth.None));
        annotation(experiment(Tolerance=1e-08, StopTime=10800),
      __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Storage/Examples/Stratified.mos"
              "Simulate and plot"),
          Documentation(info="<html>
This test model compares two tank models. The only difference between
the two tank models is that one uses the third order upwind discretization
scheme that reduces numerical diffusion that is induced when connecting
volumes in series.
</html>",       revisions="<html>
<ul>
<li>
June 7, 2018 by Filip Jorissen:<br/>
Copied model from Buildings and update the model accordingly.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/314\">#314</a>.
</li>
</ul>
</html>"));
      end Stratified;

      model StratifiedEnhancedInternalHex
        "Example showing the use of StratifiedEnhancedInternalHex"
        extends Modelica.Icons.Example;

        package MediumTan = Buildings.Media.Water "Medium in the tank";
        package MediumHex = Buildings.Media.Water "Medium in the heat exchanger";

        parameter Modelica.Units.SI.PressureDifference dpHex_nominal=2500
          "Pressure drop across the heat exchanger at nominal conditions";

        parameter Modelica.Units.SI.MassFlowRate mHex_flow_nominal=0.278
          "Mass flow rate of heat exchanger";

        ProsNet.Fluid.Building_Fluid.Sources.Boundary_pT bouWat(redeclare
            package Medium = MediumTan, nPorts=3)
          "Boundary condition for water (used to set pressure)" annotation (
            Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={72,-78})));
        ProsNet.Fluid.Building_Fluid.Sources.Boundary_pT solColSup(
          redeclare package Medium = MediumHex,
          nPorts=3,
          use_p_in=true,
          T=353.15) "Water from solar collector" annotation (Placement(
              transformation(extent={{-10,-10},{10,10}}, origin={-30,40})));
        ProsNet.Fluid.Building_Fluid.Sources.Boundary_pT toSolCol(
          redeclare package Medium = MediumHex,
          nPorts=3,
          p(displayUnit="Pa") = 3E5,
          T=283.15) "Water to solar collector" annotation (Placement(
              transformation(extent={{-10,-10},{10,10}}, origin={-72,-20})));
        ProsNet.Under_Development.Storage.Storage.StratifiedEnhancedInternalHex
          tanSte(
          redeclare package Medium = MediumTan,
          m_flow_nominal=0.001,
          VTan=0.151416,
          dIns=0.0762,
          redeclare package MediumHex = MediumHex,
          CHex=40,
          Q_flow_nominal=0.278*4200*20,
          hTan=1.746,
          hHex_a=0.995,
          hHex_b=0.1,
          energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
          energyDynamicsHex=Modelica.Fluid.Types.Dynamics.SteadyState,
          allowFlowReversal=false,
          allowFlowReversalHex=false,
          mHex_flow_nominal=mHex_flow_nominal,
          TTan_nominal=293.15,
          THex_nominal=323.15,
          dpHex_nominal=dpHex_nominal)
          "Tank with heat exchanger configured as steady state"
          annotation (Placement(transformation(extent={{6,56},{40,88}})));
        ProsNet.Fluid.Building_Fluid.Sensors.TemperatureTwoPort senTemSte(
          redeclare package Medium = MediumHex,
          allowFlowReversal=false,
          m_flow_nominal=mHex_flow_nominal,
          tau=0) "Temperature sensor for outlet of steady-state heat exchanger"
          annotation (Placement(transformation(extent={{-20,0},{-40,20}})));
        Modelica.Blocks.Sources.Step step(
          height=dpHex_nominal,
          offset=3E5,
          startTime=300) "Step input for mass flow rate"
          annotation (Placement(transformation(extent={{-80,38},{-60,58}})));
        ProsNet.Fluid.Building_Fluid.Sensors.TemperatureTwoPort senTemDyn(
          redeclare package Medium = MediumHex,
          allowFlowReversal=false,
          m_flow_nominal=mHex_flow_nominal,
          tau=0) "Temperature sensor for outlet of dynamic heat exchanger"
          annotation (Placement(transformation(extent={{-22,-30},{-42,-10}})));
        ProsNet.Under_Development.Storage.Storage.StratifiedEnhancedInternalHex
          tanDyn(
          redeclare package Medium = MediumTan,
          m_flow_nominal=0.001,
          VTan=0.151416,
          dIns=0.0762,
          redeclare package MediumHex = MediumHex,
          CHex=40,
          Q_flow_nominal=0.278*4200*20,
          hTan=1.746,
          hHex_a=0.995,
          hHex_b=0.1,
          energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
          allowFlowReversal=false,
          allowFlowReversalHex=false,
          mHex_flow_nominal=mHex_flow_nominal,
          energyDynamicsHex=Modelica.Fluid.Types.Dynamics.FixedInitial,
          TTan_nominal=293.15,
          THex_nominal=323.15) "Tank with heat exchanger configured as dynamic"
          annotation (Placement(transformation(extent={{4,-28},{38,4}})));
        ProsNet.Under_Development.Storage.Storage.StratifiedEnhancedInternalHex
          tanDynSol(
          redeclare package Medium = MediumTan,
          m_flow_nominal=0.001,
          VTan=0.151416,
          dIns=0.0762,
          redeclare package MediumHex = MediumHex,
          CHex=40,
          Q_flow_nominal=0.278*4200*20,
          hTan=1.746,
          hHex_a=0.995,
          hHex_b=0.1,
          energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
          allowFlowReversal=false,
          allowFlowReversalHex=false,
          mHex_flow_nominal=mHex_flow_nominal,
          energyDynamicsHex=Modelica.Fluid.Types.Dynamics.SteadyState,
          energyDynamicsHexSolid=Modelica.Fluid.Types.Dynamics.SteadyStateInitial,
          TTan_nominal=293.15,
          THex_nominal=323.15)
          "Tank with heat exchanger configured as steady-state except for metal which is dynamic"
          annotation (Placement(transformation(extent={{6,-76},{40,-44}})));

        ProsNet.Fluid.Building_Fluid.Sensors.TemperatureTwoPort senTemDynSol(
          redeclare package Medium = MediumHex,
          allowFlowReversal=false,
          m_flow_nominal=mHex_flow_nominal,
          tau=0)
          "Temperature sensor for outlet of steady state heat exchanger with solid configured dynamic"
          annotation (Placement(transformation(extent={{-20,-82},{-40,-62}})));
      equation
        connect(solColSup.ports[1], tanSte.portHex_a) annotation (Line(
            points={{-20,38.6667},{-4,38.6667},{-4,65.92},{6,65.92}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(senTemSte.port_b, toSolCol.ports[1])
          annotation (Line(points={{-40,10},{-40,10},{-50,10},{-50,-21.3333},{-62,
                -21.3333}},                          color={0,127,255}));
        connect(senTemSte.port_a, tanSte.portHex_b) annotation (Line(points={{-20,10},
                {0,10},{0,59.2},{6,59.2}}, color={0,127,255}));
        connect(senTemDyn.port_a, tanDyn.portHex_b) annotation (Line(points={{-22,-20},
                {0,-20},{0,-24.8},{4,-24.8}}, color={0,127,255}));
        connect(senTemDyn.port_b, toSolCol.ports[2]) annotation (Line(points={{-42,-20},
                {-52,-20},{-62,-20}},           color={0,127,255}));
        connect(tanDynSol.portHex_b, senTemDynSol.port_a) annotation (Line(points={{6,-72.8},
                {-8,-72.8},{-8,-72},{-20,-72}},        color={0,127,255}));
        connect(senTemDynSol.port_b, toSolCol.ports[3]) annotation (Line(points={{-40,-72},
                {-50,-72},{-50,-18.6667},{-62,-18.6667}},                color={0,127,
                255}));
        connect(solColSup.ports[2], tanDyn.portHex_a) annotation (Line(points={{-20,40},
                {-14,40},{-4,40},{-4,-18.08},{4,-18.08}}, color={0,127,255}));
        connect(solColSup.ports[3], tanDynSol.portHex_a) annotation (Line(points={{-20,
                41.3333},{-6,41.3333},{-6,-66.08},{6,-66.08}},               color={0,
                127,255}));
        connect(bouWat.ports[1], tanSte.port_b) annotation (Line(points={{62,-79.3333},
                {52,-79.3333},{52,40},{22,40},{22,56},{23,56}},
                                                color={0,127,255}));
        connect(bouWat.ports[2], tanDyn.port_b)
          annotation (Line(points={{62,-78},{52,-78},{52,-36},{21,-36},{21,-28}},
                                                                color={0,127,255}));
        connect(bouWat.ports[3], tanDynSol.port_b) annotation (Line(points={{62,
                -76.6667},{24,-76.6667},{24,-76},{23,-76}},
                                                  color={0,127,255}));
        connect(step.y, solColSup.p_in)
          annotation (Line(points={{-59,48},{-52,48},{-42,48}}, color={0,0,127}));
        annotation ( __Dymola_Commands(file=
                "modelica://Buildings/Resources/Scripts/Dymola/Fluid/Storage/Examples/StratifiedEnhancedInternalHex.mos"
              "Simulate and plot"),
      experiment(Tolerance=1e-6, StopTime=1200),
      Documentation(info="<html>
<p>
This model provides an example for the
<a href=\"modelica://Buildings.Fluid.Storage.StratifiedEnhancedInternalHex\">
Buildings.Fluid.Storage.StratifiedEnhancedInternalHex</a> model.
There are three tanks.
In the tank on top, the fluid in the heat exchanger and the metal of the
heat exchanger use a steady-state energy balance.
In the middle tank, both use a dynamic balance.
In the bottom tank, the fluid uses a steady-state heat balance
but the metal of the heat exchanger uses a dynamic balance.
</p>
<p>
Each tank starts at the same water temperature, and there is no
water flow through the tank.
The glycol that flows through the heat exchanger starts with zero
mass flow rate, and is set to its design flow rate at <i>t=300</i> seconds.
</p>
</html>",
      revisions="<html>
<ul>
<li>
June 7, 2018 by Filip Jorissen:<br/>
Copied model from Buildings and update the model accordingly.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/314\">#314</a>.
</li>
<li>
January 22, 2016, by Michael Wetter:<br/>
Corrected type declaration of pressure difference.
This is
for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/404\">#404</a>.
</li>
<li>
September 28, 2015 by Michael Wetter:<br/>
Changed medium in heat exchanger from
<a href=\"modelica://Modelica.Media.Incompressible.Examples.Glycol47\">
Modelica.Media.Incompressible.Examples.Glycol47</a> to
<a href=\"modelica://Buildings.Media.Water\">
Buildings.Media.Water</a>
to avoid numerical derivative in regression tests.
</li>
<li>
July 2, 2015 by Michael Wetter:<br/>
Modified example to test dynamic versus steady-state heat exchanger
configuration.
</li>
<li>
December 22, 2014 by Michael Wetter:<br/>
Removed <code>Modelica.Fluid.System</code>
to address issue
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/311\">#311</a>.
</li>
<li>
August 29, 2014 by Michael Wetter:<br/>
Revised example to use a different media in the tank and in the
heat exchanger. This is to provide a unit test for
issue <a href=\"https://github.com/lbl-srg/modelica-buildings/issues/271\">#271</a>.
</li>
<li>
April 18, 2014 by Michael Wetter:<br/>
Revised example for new connectors and parameters, and provided
more interesting parameter values that cause a tank stratification.
</li>
<li>
Mar 27, 2013 by Peter Grant:<br/>
First implementation
</li>
</ul>
</html>"));
      end StratifiedEnhancedInternalHex;

      model StratifiedUnloadAtMinimumTemperature
        "Example that demonstrates how to draw from a hot water tank at the minimum temperature"
        extends Modelica.Icons.Example;
        package Medium = Buildings.Media.Water "Medium model";

        parameter Modelica.Units.SI.Volume VTan=3 "Tank volume";

        parameter Modelica.Units.SI.MassFlowRate m_flow_nominal=3*1000/3600
          "Nominal mass flow rate";

        constant Integer nSeg=5 "Number of volume segments";

        ProsNet.Under_Development.Storage.Storage.Stratified tan(
          redeclare package Medium = Medium,
          m_flow_nominal=m_flow_nominal,
          VTan=VTan,
          hTan=2,
          dIns=0.2,
          energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
          nSeg=nSeg,
          T_start=353.15) "Hot water storage tank" annotation (Placement(
              transformation(extent={{-120,-128},{-100,-108}})));
        ProsNet.Fluid.Building_Fluid.Sources.Boundary_pT loa(redeclare package
            Medium = Medium, nPorts=1)
          "Load (imposed by a constant pressure boundary condition and the flow of masSou)"
          annotation (Placement(transformation(extent={{242,-70},{222,-50}})));
        ProsNet.Fluid.Building_Fluid.Sources.MassFlowSource_T masSou(
          nPorts=1,
          redeclare package Medium = Medium,
          m_flow=m_flow_nominal) "Mass flow rate into the tank" annotation (
            Placement(transformation(extent={{242,-130},{222,-110}})));

        ProsNet.Fluid.Building_Fluid.Actuators.Valves.TwoWayLinear valTop(
          redeclare package Medium = Medium,
          m_flow_nominal=m_flow_nominal,
          dpValve_nominal=3000,
          use_inputFilter=false) "Control valve at top"
          annotation (Placement(transformation(extent={{112,-30},{132,-10}})));

        ProsNet.Fluid.Building_Fluid.Actuators.Valves.TwoWayLinear valMid(
          redeclare package Medium = Medium,
          m_flow_nominal=m_flow_nominal,
          dpValve_nominal=3000,
          use_inputFilter=false) "Control valve at middle"
          annotation (Placement(transformation(extent={{132,-70},{152,-50}})));

        ProsNet.Fluid.Building_Fluid.Actuators.Valves.TwoWayLinear valBot(
          redeclare package Medium = Medium,
          m_flow_nominal=m_flow_nominal,
          dpValve_nominal=3000,
          use_inputFilter=false) "Control valve at bottom"
          annotation (Placement(transformation(extent={{150,-110},{170,-90}})));

        Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor TMid
          "Temperature tank middle"
          annotation (Placement(transformation(extent={{-100,70},{-80,90}})));
        Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor TBot
          "Temperature tank bottom"
          annotation (Placement(transformation(extent={{-100,30},{-80,50}})));
        Modelica.Blocks.Logical.Hysteresis onOffBot(uLow=273.15 + 40 - 0.05, uHigh=
              273.15 + 40 + 0.05)
          "Controller for valve at bottom"
          annotation (Placement(transformation(extent={{-50,30},{-30,50}})));
        ProsNet.Fluid.Building_Fluid.Sensors.TemperatureTwoPort senTem(
          redeclare package Medium = Medium,
          m_flow_nominal=m_flow_nominal,
          tau=0) "Outflowing temperature"
          annotation (Placement(transformation(extent={{190,-70},{210,-50}})));
        Modelica.Blocks.Logical.Hysteresis onOffMid(uLow=273.15 + 40 - 0.05, uHigh=
              273.15 + 40 + 0.05)
          "Controller for valve at middle of tank"
          annotation (Placement(transformation(extent={{-50,70},{-30,90}})));
        Modelica.Blocks.Logical.And and2
          "And block to compute control action for middle valve"
          annotation (Placement(transformation(extent={{10,70},{30,90}})));
        Modelica.Blocks.Math.BooleanToReal yMid
          "Boolean to real conversion for valve at middle"
          annotation (Placement(transformation(extent={{80,70},{100,90}})));
        Modelica.Blocks.Math.BooleanToReal yTop
          "Boolean to real conversion for valve at top"
          annotation (Placement(transformation(extent={{80,110},{100,130}})));
        Modelica.Blocks.Logical.Nor nor
          "Nor block for top-most control valve"
          annotation (Placement(transformation(extent={{50,110},{70,130}})));
        Modelica.Blocks.Logical.Not not1
          "Not block to negate control action of upper level control"
          annotation (Placement(transformation(extent={{-20,50},{0,70}})));
        Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow hea
          "Heat input at the bottom of the tank"
          annotation (Placement(transformation(extent={{-150,-134},{-130,-114}})));
        Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor TTop
          "Temperature tank top"
          annotation (Placement(transformation(extent={{-160,-100},{-180,-80}})));
        Modelica.Blocks.Logical.Hysteresis onOffHea(uLow=273.15 + 50 - 0.05, uHigh=
              273.15 + 50 + 0.05)
          "Controller for heater at bottom"
          annotation (Placement(transformation(extent={{-210,-134},{-190,-114}})));
        Modelica.Blocks.Math.BooleanToReal yHea(realFalse=150000)
          "Boolean to real for valve at bottom"
          annotation (Placement(transformation(extent={{-180,-134},{-160,-114}})));
        Modelica.Blocks.Math.BooleanToReal yBot
          "Boolean to real conversion for valve at bottom"
          annotation (Placement(transformation(extent={{80,30},{100,50}})));
      equation
        connect(masSou.ports[1], tan.port_b) annotation (Line(points={{222,-120},{56,
                -120},{56,-128},{-110,-128}},
                                    color={0,127,255}));
        connect(TMid.port, tan.heaPorVol[3])
          annotation (Line(points={{-100,80},{-100,-118},{-110,-118}},
                                                                   color={191,0,0}));
        connect(TBot.port, tan.heaPorVol[5])
          annotation (Line(points={{-100,40},{-100,-117.76},{-110,-117.76}},
                                                                 color={191,0,0}));
        connect(valTop.port_b, senTem.port_a) annotation (Line(points={{132,-20},{182,
                -20},{182,-60},{190,-60}},
                                         color={0,127,255}));
        connect(valMid.port_b, senTem.port_a)
          annotation (Line(points={{152,-60},{190,-60}},
                                                       color={0,127,255}));
        connect(valBot.port_b, senTem.port_a) annotation (Line(points={{170,-100},{182,
                -100},{182,-60},{190,-60}},
                                    color={0,127,255}));
        connect(senTem.port_b,loa. ports[1])
          annotation (Line(points={{210,-60},{222,-60}},
                                                       color={0,127,255}));
        connect(valTop.port_a, tan.fluPorVol[1]) annotation (Line(points={{112,-20},{
                -116,-20},{-116,-118},{-112.6,-118},{-112.6,-118.4}},
                                     color={0,127,255}));
        connect(valMid.port_a, tan.fluPorVol[3]) annotation (Line(points={{132,-60},{
                -116,-60},{-116,-118},{-112.6,-118}},
                                    color={0,127,255}));
        connect(valBot.port_a, tan.fluPorVol[5]) annotation (Line(points={{150,-100},
                {-116,-100},{-116,-118},{-112.6,-118},{-112.6,-117.6}},
                                    color={0,127,255}));
        connect(onOffMid.y, and2.u1)
          annotation (Line(points={{-29,80},{8,80}},     color={255,0,255}));
        connect(yMid.u, and2.y)
          annotation (Line(points={{78,80},{31,80}},   color={255,0,255}));
        connect(yTop.u, nor.y)
          annotation (Line(points={{78,120},{71,120}},
                                                   color={255,0,255}));
        connect(and2.y, nor.u1) annotation (Line(points={{31,80},{40,80},{40,120},{48,
                120}},
              color={255,0,255}));
        connect(onOffBot.y, nor.u2) annotation (Line(points={{-29,40},{46,40},{46,112},
                {48,112}},color={255,0,255}));
        connect(yTop.y, valTop.y) annotation (Line(points={{101,120},{122,120},{122,-8}},
                                     color={0,0,127}));
        connect(yMid.y, valMid.y) annotation (Line(points={{101,80},{142,80},{142,-48}},
                                    color={0,0,127}));
        connect(not1.u, onOffBot.y) annotation (Line(points={{-22,60},{-26,60},{-26,40},
                {-29,40}},       color={255,0,255}));
        connect(not1.y, and2.u2) annotation (Line(points={{1,60},{4,60},{4,72},{8,72}},
              color={255,0,255}));
        connect(hea.port, tan.heaPorVol[5]) annotation (Line(points={{-130,-124},{
                -110,-124},{-110,-117.76}},
                                    color={191,0,0}));
        connect(TTop.port, tan.heaPorVol[1]) annotation (Line(points={{-160,-90},{
                -120,-90},{-120,-120},{-110,-120},{-110,-118.24}},
                                   color={191,0,0}));
        connect(onOffHea.u, TTop.T) annotation (Line(points={{-212,-124},{-230,-124},
                {-230,-90},{-181,-90}},color={0,0,127}));
        connect(onOffHea.y, yHea.u)
          annotation (Line(points={{-189,-124},{-182,-124}}, color={255,0,255}));
        connect(hea.Q_flow, yHea.y)
          annotation (Line(points={{-150,-124},{-159,-124}}, color={0,0,127}));
        connect(onOffBot.y, yBot.u)
          annotation (Line(points={{-29,40},{78,40}}, color={255,0,255}));
        connect(yBot.y, valBot.y)
          annotation (Line(points={{101,40},{160,40},{160,-88}}, color={0,0,127}));
        connect(TBot.T, onOffBot.u)
          annotation (Line(points={{-79,40},{-52,40}}, color={0,0,127}));
        connect(onOffMid.u, TMid.T)
          annotation (Line(points={{-52,80},{-79,80}}, color={0,0,127}));
        annotation (Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-300,-140},{260,140}})),
             __Dymola_Commands(file=
                "modelica://Buildings/Resources/Scripts/Dymola/Fluid/Storage/Examples/StratifiedUnloadAtMinimumTemperature.mos"
              "Simulate and plot"),
          experiment(
            StopTime=21600,
            Tolerance=1e-06),
          Documentation(info="<html>
<p>
Example for tank model that has three outlets, each with a valve.
The valve at the bottom opens when the temperature in that tank segment
is sufficiently warm to serve the load.
The tank in the middle also opens when its tank temperature is sufficiently high,
but only if the valve below is closed.
Finally, the valve at the top only opens if no other valve is open.
Hence, there is always exactly one valve open.
On the right-hand side of the model is a heater that adds heat to the bottom of the
tank if the top tank segment is below the set point temperature.
</p>
</html>",       revisions="<html>
<ul>
<li>
June 7, 2018 by Filip Jorissen:<br/>
Copied model from Buildings and update the model accordingly
by removing CDL blocks.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/314\">#314</a>.
</li>
<li>
June 1, 2018, by Michael Wetter:<br/>
First implementation.<br/>
This is for <a href=\"https://github.com/lbl-srg/modelica-buildings/issues/1182\">
issue 1182</a>.
</li>
</ul>
</html>"));
      end StratifiedUnloadAtMinimumTemperature;
    annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains examples for the use of models that can be found in
<a href=\"modelica://Buildings.Fluid.Storage\">
Buildings.Fluid.Storage</a>.
</p>
</html>"));
    end Examples;

    package Validation "Collection of models that validate the storage models"
    extends Modelica.Icons.ExamplesPackage;

      model HeatExchangerDynamics
        "Test model for stratified tank with steady-state and dynamic heat exchanger balance"
        extends Modelica.Icons.Example;

        package Medium = Buildings.Media.Water "Medium model";

        constant Integer nSeg = 7 "Number of segments in tank";

        parameter Modelica.Units.SI.HeatFlowRate QHex_flow_nominal=2000
          "Design heat flow rate of heat exchanger";
        parameter Modelica.Units.SI.MassFlowRate m_flow_nominal=QHex_flow_nominal/
            4200/4;

        ProsNet.Fluid.Building_Fluid.Sources.Boundary_pT watInTan(
          redeclare package Medium = Medium,
          use_T_in=false,
          nPorts=2,
          T=273.15 + 30,
          p(displayUnit="Pa")) "Boundary condition for water in the tank"
          annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
        ProsNet.Fluid.Building_Fluid.Sources.MassFlowSource_T mHex_flow1(
          redeclare package Medium = Medium,
          use_m_flow_in=true,
          T=273.15 + 60,
          nPorts=1) "Mass flow rate through the heat exchanger"
          annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
        model Tank = StratifiedEnhancedInternalHex (
          redeclare package Medium = Medium,
          redeclare package MediumHex = Medium,
          hTan=2,
          dIns=0.3,
          VTan=0.3,
          nSeg=nSeg,
          hHex_a=1,
          hHex_b=0.2,
          Q_flow_nominal=QHex_flow_nominal,
          TTan_nominal=313.15,
          THex_nominal=333.15,
          mHex_flow_nominal=m_flow_nominal,
          show_T=true,
          m_flow_nominal=m_flow_nominal,
          energyDynamicsHex=Modelica.Fluid.Types.Dynamics.FixedInitial)
          "Tank with dynamic heat exchanger balance";

        Tank tanDyn "Tank with dynamic heat exchanger balance"
          annotation (Placement(transformation(extent={{32,20},{52,40}})));

        Tank tanSte(energyDynamicsHex=Modelica.Fluid.Types.Dynamics.SteadyState)
          "Tank with steady-state heat exchanger balance"
          annotation (Placement(transformation(extent={{32,-20},{52,0}})));

        Modelica.Blocks.Sources.Trapezoid mHex_flow_in(
          period=7200,
          amplitude=m_flow_nominal,
          offset=0,
          rising=1800,
          width=1800,
          falling=1800,
          startTime=900) "Control signal for mass flow rate in heat exchanger"
          annotation (Placement(transformation(extent={{-100,10},{-80,30}})));
        ProsNet.Fluid.Building_Fluid.Sources.Boundary_pT sin(
          redeclare package Medium = Medium,
          T=273.15 + 30,
          p(displayUnit="Pa"),
          nPorts=2) "Sink boundary condition"
          annotation (Placement(transformation(extent={{-62,-48},{-42,-28}})));

        ProsNet.Fluid.Building_Fluid.Sources.MassFlowSource_T mHex_flow2(
          redeclare package Medium = Medium,
          use_m_flow_in=true,
          T=273.15 + 60,
          nPorts=1) "Mass flow rate through the heat exchanger"
          annotation (Placement(transformation(extent={{-60,-20},{-40,0}})));
        ProsNet.Fluid.Building_Fluid.Sensors.TemperatureTwoPort senTanDyn(
          redeclare package Medium = Medium,
          allowFlowReversal=false,
          m_flow_nominal=m_flow_nominal,
          tau=0) "Temperature sensor at tank outlet"
          annotation (Placement(transformation(extent={{10,0},{-10,20}})));
        ProsNet.Fluid.Building_Fluid.Sensors.TemperatureTwoPort senTanSte(
          redeclare package Medium = Medium,
          allowFlowReversal=false,
          m_flow_nominal=m_flow_nominal,
          tau=0) "Temperature sensor at tank outlet"
          annotation (Placement(transformation(extent={{10,-50},{-10,-30}})));
        ProsNet.Fluid.Building_Fluid.Sources.MassFlowSource_T mWatTanDyn_flow(
            redeclare package Medium = Medium, nPorts=1)
          "Mass flow rate through the tank"
          annotation (Placement(transformation(extent={{82,10},{62,30}})));
        ProsNet.Fluid.Building_Fluid.Sources.MassFlowSource_T mWatTanSte_flow(
            redeclare package Medium = Medium, nPorts=1)
          "Mass flow rate through the tank"
          annotation (Placement(transformation(extent={{82,-30},{62,-10}})));
      equation
        connect(mHex_flow_in.y, mHex_flow1.m_flow_in) annotation (Line(
            points={{-79,20},{-70,20},{-70,28},{-62,28}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(mHex_flow2.m_flow_in, mHex_flow_in.y) annotation (Line(points={{-62,-2},
                {-70,-2},{-70,20},{-79,20}}, color={0,0,127}));
        connect(mHex_flow2.ports[1], tanSte.portHex_a) annotation (Line(points={{-40,-10},
                {-32,-10},{-30,-10},{-30,-13.8},{32,-13.8}},  color={0,127,255}));
        connect(watInTan.ports[1], tanSte.port_a) annotation (Line(points={{-40,59},{
                -40,59},{-22,59},{-22,0},{42,0}},  color={0,127,255}));
        connect(mHex_flow1.ports[1], tanDyn.portHex_a) annotation (Line(points={{-40,
                20},{-30,20},{-30,26.2},{32,26.2}}, color={0,127,255}));
        connect(watInTan.ports[2], tanDyn.port_a) annotation (Line(points={{-40,61},{
                -30,61},{-20,61},{-20,40},{42,40}},
                                                 color={0,127,255}));
        connect(senTanDyn.port_a, tanDyn.portHex_b) annotation (Line(points={{10,10},{
                20,10},{20,22},{32,22}}, color={0,127,255}));
        connect(senTanSte.port_a, tanSte.portHex_b) annotation (Line(points={{10,-40},
                {20,-40},{20,-18},{32,-18}}, color={0,127,255}));
        connect(senTanDyn.port_b, sin.ports[1]) annotation (Line(points={{-10,10},{
                -20,10},{-20,-39},{-42,-39}},
                                          color={0,127,255}));
        connect(senTanSte.port_b, sin.ports[2]) annotation (Line(points={{-10,-40},{
                -42,-40},{-42,-37}},
                                 color={0,127,255}));
        connect(mWatTanDyn_flow.ports[1], tanDyn.port_b)
          annotation (Line(points={{62,20},{42,20}}, color={0,127,255}));
        connect(mWatTanSte_flow.ports[1], tanSte.port_b)
          annotation (Line(points={{62,-20},{42,-20}}, color={0,127,255}));
        annotation (
        __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Storage/Validation/HeatExchangerDynamics.mos"
              "Simulate and plot"),
          Documentation(info="<html>
This validation model compares two tank models. The only difference between
the two tank models is that one uses a dynamic energy balance, whereas
the other uses a steady-state energy balance for the heat exchanger.
The mass flow rate through the heat exchanger is varied from zero to
the design flow rate and back to zero to test the model under conditions in
which no water flows through the heat exchanger.
</html>",       revisions="<html>
<ul>
<li>
June 7, 2018 by Filip Jorissen:<br/>
Copied model from Buildings and update the model accordingly.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/314\">#314</a>.
</li>
<li>
July 5, 2017, by Michael Wetter:<br/>
Added zero mass flow rate boundary conditions to avoid a translation error in Dymola 2018.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/834\">issue 834</a>.
</li>
<li>
January 8, 2016 by Michael Wetter:<br/>
First implementation to test
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/434\">issue 434</a>.
</li>
</ul>
</html>"),experiment(Tolerance=1e-6, StopTime=14400));
      end HeatExchangerDynamics;

      model HeatExchangerLocation
        "Test model for heat exchanger with hHex_a and hHex_b interchanged"
        extends Modelica.Icons.Example;

        package Medium = Buildings.Media.Water "Medium model";

        parameter Modelica.Units.SI.HeatFlowRate QHex_flow_nominal=6000
          "Design heat flow rate of heat exchanger";
        parameter Modelica.Units.SI.MassFlowRate m_flow_nominal=QHex_flow_nominal/
            4200/4;

        ProsNet.Fluid.Building_Fluid.Sources.Boundary_pT watInTan(
          redeclare package Medium = Medium,
          use_T_in=false,
          nPorts=2,
          T=273.15 + 30,
          p(displayUnit="Pa")) "Boundary condition for water in the tank"
          annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
        ProsNet.Fluid.Building_Fluid.Sources.MassFlowSource_T mHex_flow1(
          redeclare package Medium = Medium,
          nPorts=2,
          m_flow=m_flow_nominal/100,
          T=273.15 + 60) "Mass flow rate through the heat exchanger"
          annotation (Placement(transformation(extent={{-40,10},{-20,30}})));
        model Tank = StratifiedEnhancedInternalHex (
          redeclare final package Medium = Medium,
          redeclare final package MediumHex = Medium,
          final VTan=0.3,
          final kIns=0.034,
          final dIns=0.04,
          final hTan=1.2,
          final m_flow_nominal=m_flow_nominal,
          final mHex_flow_nominal=m_flow_nominal,
          final nSeg=12,
          final hexSegMult=2,
          final Q_flow_nominal=QHex_flow_nominal,
          final TTan_nominal=293.15,
          final THex_nominal=273.15+60,
          final computeFlowResistance=false,
          final energyDynamicsHex=Modelica.Fluid.Types.Dynamics.FixedInitial)
          "Tank";

        Tank tan_aTop(
          hHex_a=0.3,
          hHex_b=0.1)
          "Tank with heat exchanger inlet above its outlet"
          annotation (Placement(transformation(extent={{42,14},{62,34}})));

        Tank tan_bTop(
          hHex_a=0.1,
          hHex_b=0.3)
          "Tank with heat exchanger outlet above its inlet"
          annotation (Placement(transformation(extent={{40,-58},{60,-38}})));

        ProsNet.Fluid.Building_Fluid.Sources.Boundary_pT sin(
          redeclare package Medium = Medium,
          T=273.15 + 30,
          p(displayUnit="Pa"),
          nPorts=2) "Sink boundary condition"
          annotation (Placement(transformation(extent={{-60,-32},{-40,-12}})));

        ProsNet.Fluid.Building_Fluid.Sensors.TemperatureTwoPort senTan_aTop(
          redeclare package Medium = Medium,
          m_flow_nominal=m_flow_nominal,
          tau=0) "Temperature sensor at tank outlet"
          annotation (Placement(transformation(extent={{10,-30},{-10,-10}})));

        ProsNet.Fluid.Building_Fluid.Sensors.TemperatureTwoPort senTan_bTop(
          redeclare package Medium = Medium,
          m_flow_nominal=m_flow_nominal,
          tau=0) "Temperature sensor at tank outlet"
          annotation (Placement(transformation(extent={{10,-80},{-10,-60}})));
        ProsNet.Fluid.Building_Fluid.Sources.MassFlowSource_T mWatTanSte_flow(
            redeclare package Medium = Medium, nPorts=1)
          "Mass flow rate through the tank"
          annotation (Placement(transformation(extent={{92,-58},{72,-38}})));
        ProsNet.Fluid.Building_Fluid.Sources.MassFlowSource_T mWatTanDyn_flow(
            redeclare package Medium = Medium, nPorts=1)
          "Mass flow rate through the tank"
          annotation (Placement(transformation(extent={{94,14},{74,34}})));
      equation
        connect(mHex_flow1.ports[1], tan_aTop.portHex_a) annotation (Line(points={{-20,22},
                {-20,20.2},{42,20.2}},     color={0,127,255}));
        connect(senTan_aTop.port_a, tan_aTop.portHex_b) annotation (Line(points={{10,-20},
                {10,-20},{20,-20},{20,16},{42,16}}, color={0,127,255}));
        connect(senTan_aTop.port_b, sin.ports[1]) annotation (Line(points={{-10,-20},{
                -26,-20},{-40,-20}}, color={0,127,255}));
        connect(senTan_bTop.port_b, sin.ports[2]) annotation (Line(points={{-10,-70},{
                -20,-70},{-20,-24},{-40,-24}}, color={0,127,255}));
        connect(senTan_bTop.port_a, tan_bTop.portHex_b) annotation (Line(points={{10,-70},
                {20,-70},{20,-56},{40,-56}}, color={0,127,255}));
        connect(tan_bTop.portHex_a, mHex_flow1.ports[2]) annotation (Line(points={{40,
                -51.8},{30,-51.8},{30,18},{-20,18}}, color={0,127,255}));
        connect(tan_aTop.port_a, watInTan.ports[1]) annotation (Line(points={{42,24},{
                38,24},{38,62},{-40,62}}, color={0,127,255}));
        connect(tan_bTop.port_a, watInTan.ports[2]) annotation (Line(points={{40,-48},
                {40,-48},{34,-48},{34,58},{-40,58}}, color={0,127,255}));
        connect(mWatTanDyn_flow.ports[1], tan_aTop.port_b)
          annotation (Line(points={{74,24},{62,24}}, color={0,127,255}));
        connect(tan_bTop.port_b, mWatTanSte_flow.ports[1])
          annotation (Line(points={{60,-48},{72,-48}}, color={0,127,255}));
        annotation (
        __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Storage/Validation/HeatExchangerLocation.mos"
              "Simulate and plot"),
          Documentation(info="<html>
<p>
This validation model compares two tank models. The only difference between
the two tank models is that <code>tan_aTop</code> has the hot water inlet
for the heat exchanger above its outlet, whereas <code>tan_bTop</code>
has the hot water inlet below its outlet. In both models, the heat exchanger
extends from element <i>9</i> to element <i>11</i>.
</p>
</html>",       revisions="<html>
<ul>
<li>
June 7, 2018 by Filip Jorissen:<br/>
Copied model from Buildings and update the model accordingly.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/314\">#314</a>.
</li>
<li>
July 5, 2017, by Michael Wetter:<br/>
Added zero mass flow rate boundary conditions to avoid a translation error in Dymola 2018.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/834\">issue 834</a>.
</li>
<li>
June 23, 2016 by Michael Wetter:<br/>
First implementation to test
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/531\">issue 531</a>.
</li>
</ul>
</html>"),experiment(Tolerance=1e-6, StopTime=600000));
      end HeatExchangerLocation;

      model StratifiedLoadingUnloading "Test model for stratified tank"
        extends Modelica.Icons.Example;

       package Medium = Buildings.Media.Water "Medium model";
       constant Integer nSeg = 7 "Number of segments in tank";

        parameter Modelica.Units.SI.MassFlowRate m_flow_nominal=1*1000/3600/4;

        ProsNet.Fluid.Building_Fluid.Sources.Boundary_pT sou_1(
          p=300000 + 5000,
          T=273.15 + 40,
          redeclare package Medium = Medium,
          use_T_in=false,
          nPorts=2)
          annotation (Placement(transformation(extent={{-100,-20},{-80,0}})));
        ProsNet.Fluid.Building_Fluid.Sources.MassFlowSource_T sin_1(
          redeclare package Medium = Medium,
          T=273.15 + 20,
          m_flow=-0.028,
          use_m_flow_in=true,
          nPorts=1)
          annotation (Placement(transformation(extent={{78,-2},{58,18}})));
        ProsNet.Under_Development.Storage.Storage.StratifiedEnhanced tanEnh(
          redeclare package Medium = Medium,
          hTan=3,
          dIns=0.3,
          VTan=0.1,
          nSeg=nSeg,
          show_T=true,
          m_flow_nominal=m_flow_nominal) "Tank"
          annotation (Placement(transformation(extent={{-30,-2},{-10,18}})));

        ProsNet.Fluid.Building_Fluid.Sources.MassFlowSource_T sin_2(
          redeclare package Medium = Medium,
          T=273.15 + 20,
          m_flow=-0.028,
          use_m_flow_in=true,
          nPorts=1)
          annotation (Placement(transformation(extent={{78,-40},{58,-20}})));
        ProsNet.Under_Development.Storage.Storage.Stratified tan(
          redeclare package Medium = Medium,
          hTan=3,
          dIns=0.3,
          VTan=0.1,
          nSeg=nSeg,
          show_T=true,
          m_flow_nominal=m_flow_nominal) "Tank"
          annotation (Placement(transformation(extent={{-26,-40},{-6,-20}})));

        Modelica.Blocks.Sources.Pulse pulse(
          amplitude=2*m_flow_nominal,
          offset=-m_flow_nominal,
          period=7200)
          annotation (Placement(transformation(extent={{20,80},{40,100}})));
        ProsNet.Fluid.Building_Fluid.Sensors.EnthalpyFlowRate HIn_flow(
            redeclare package Medium = Medium, m_flow_nominal=m_flow_nominal)
          "Enthalpy flow rate"
          annotation (Placement(transformation(extent={{-60,-38},{-44,-22}})));
        ProsNet.Fluid.Building_Fluid.Sensors.EnthalpyFlowRate HOut_flow(
            redeclare package Medium = Medium, m_flow_nominal=m_flow_nominal)
          "Enthalpy flow rate"
          annotation (Placement(transformation(extent={{22,-38},{38,-22}})));
        ProsNet.Fluid.Building_Fluid.Sensors.EnthalpyFlowRate HInEnh_flow(
            redeclare package Medium = Medium, m_flow_nominal=m_flow_nominal)
          "Enthalpy flow rate"
          annotation (Placement(transformation(extent={{-60,0},{-44,16}})));
        ProsNet.Fluid.Building_Fluid.Sensors.EnthalpyFlowRate HOutEnh_flow(
            redeclare package Medium = Medium, m_flow_nominal=m_flow_nominal)
          "Enthalpy flow rate"
          annotation (Placement(transformation(extent={{2,0},{18,16}})));
        Modelica.Blocks.Math.Add add(k2=-1) annotation (Placement(transformation(
                extent={{20,40},{40,60}})));
        Modelica.Blocks.Continuous.Integrator dHTanEnh
          "Difference in enthalpy (should be zero at steady-state)"
          annotation (Placement(transformation(extent={{60,40},{80,60}})));
        Modelica.Blocks.Math.Add add1(
                                     k2=-1) annotation (Placement(transformation(
                extent={{20,-80},{40,-60}})));
        Modelica.Blocks.Continuous.Integrator dHTan
          "Difference in enthalpy (should be zero at steady-state)"
          annotation (Placement(transformation(extent={{60,-80},{80,-60}})));
      equation

        connect(sou_1.ports[1], HIn_flow.port_a) annotation (Line(
            points={{-80,-8},{-70,-8},{-70,-30},{-60,-30}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(HIn_flow.port_b, tan.port_a) annotation (Line(
            points={{-44,-30},{-26,-30}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(tan.port_b, HOut_flow.port_a) annotation (Line(
            points={{-6,-30},{22,-30}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(HOut_flow.port_b, sin_2.ports[1]) annotation (Line(
            points={{38,-30},{58,-30}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(sou_1.ports[2], HInEnh_flow.port_a) annotation (Line(
            points={{-80,-12},{-76,-12},{-76,-6},{-72,-6},{-72,8},{-60,8}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(HInEnh_flow.port_b, tanEnh.port_a) annotation (Line(
            points={{-44,8},{-30,8}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(tanEnh.port_b, HOutEnh_flow.port_a) annotation (Line(
            points={{-10,8},{2,8}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(HOutEnh_flow.port_b, sin_1.ports[1]) annotation (Line(
            points={{18,8},{58,8}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(HInEnh_flow.H_flow, add.u1) annotation (Line(
            points={{-52,16.8},{-52,56},{18,56}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(HOutEnh_flow.H_flow, add.u2) annotation (Line(
            points={{10,16.8},{10,44},{18,44}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(add.y, dHTanEnh.u) annotation (Line(
            points={{41,50},{58,50}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(HIn_flow.H_flow, add1.u1) annotation (Line(
            points={{-52,-21.2},{-52,-14},{-34,-14},{-34,-64},{18,-64}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(HOut_flow.H_flow, add1.u2) annotation (Line(
            points={{30,-21.2},{30,-16},{6,-16},{6,-76},{18,-76}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(add1.y, dHTan.u) annotation (Line(
            points={{41,-70},{58,-70}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(pulse.y, sin_1.m_flow_in) annotation (Line(
            points={{41,90},{92,90},{92,16},{80,16}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(pulse.y, sin_2.m_flow_in) annotation (Line(
            points={{41,90},{90,90},{90,-22},{80,-22}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (                     __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Storage/Validation/StratifiedLoadingUnloading.mos"
              "Simulate and plot"),
          Documentation(info="<html>
This test model compares two tank models. The only difference between
the two tank models is that one uses the third order upwind discretization
scheme that reduces numerical diffusion that is induced when connecting
volumes in series.
</html>",       revisions="<html>
<ul>
<li>
June 7, 2018 by Filip Jorissen:<br/>
Copied model from Buildings and update the model accordingly.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/314\">#314</a>.
</li>
</ul>
</html>"),experiment(Tolerance=1e-6, StopTime=14400));
      end StratifiedLoadingUnloading;

      model StratifiedNonUniformInitial
        "Test model for stratified tank with non-uniform initial temperature"
        extends Modelica.Icons.Example;

        package Medium = Buildings.Media.Water "Medium model";
        constant Integer nSeg = 7 "Number of segments in tank";

        parameter Modelica.Units.SI.MassFlowRate m_flow_nominal=1*1000/3600/4;

        ProsNet.Fluid.Building_Fluid.Sources.Boundary_pT sou_1(
          p=300000 + 5000,
          T=273.15 + 40,
          redeclare package Medium = Medium,
          use_T_in=false,
          nPorts=2)
          annotation (Placement(transformation(extent={{-100,-20},{-80,0}})));
        ProsNet.Fluid.Building_Fluid.Sources.MassFlowSource_T sin_1(
          redeclare package Medium = Medium,
          T=273.15 + 20,
          m_flow=-m_flow_nominal,
          nPorts=1) "Mass flow source"
          annotation (Placement(transformation(extent={{80,10},{60,30}})));
        ProsNet.Under_Development.Storage.Storage.Stratified heaTan(
          redeclare package Medium = Medium,
          hTan=3,
          dIns=0.3,
          VTan=0.1,
          nSeg=nSeg,
          show_T=true,
          m_flow_nominal=m_flow_nominal,
          TFlu_start={313.15,312.15,311.15,310.15,309.15,308.15,307.15})
          "Tank that will be heated up"
          annotation (Placement(transformation(extent={{-30,10},{-10,30}})));
        ProsNet.Fluid.Building_Fluid.Sources.MassFlowSource_T sin_2(
          redeclare package Medium = Medium,
          T=273.15 + 20,
          m_flow=m_flow_nominal,
          nPorts=1) "Mass flow source"
          annotation (Placement(transformation(extent={{80,-60},{60,-40}})));
        ProsNet.Under_Development.Storage.Storage.Stratified cooTan(
          redeclare package Medium = Medium,
          hTan=3,
          dIns=0.3,
          VTan=0.1,
          nSeg=nSeg,
          show_T=true,
          m_flow_nominal=m_flow_nominal,
          TFlu_start={313.15,312.15,311.15,310.15,309.15,308.15,307.15})
          "Tank that will be cooled down"
          annotation (Placement(transformation(extent={{-30,-60},{-10,-40}})));

        ProsNet.Fluid.Building_Fluid.Sensors.EnthalpyFlowRate HIn_flow(
            redeclare package Medium = Medium, m_flow_nominal=m_flow_nominal)
          "Enthalpy flow rate"
          annotation (Placement(transformation(extent={{-58,-58},{-42,-42}})));
        ProsNet.Fluid.Building_Fluid.Sensors.EnthalpyFlowRate HOut_flow(
            redeclare package Medium = Medium, m_flow_nominal=m_flow_nominal)
          "Enthalpy flow rate"
          annotation (Placement(transformation(extent={{2,-58},{18,-42}})));
        ProsNet.Fluid.Building_Fluid.Sensors.EnthalpyFlowRate HInEnh_flow(
            redeclare package Medium = Medium, m_flow_nominal=m_flow_nominal)
          "Enthalpy flow rate"
          annotation (Placement(transformation(extent={{-60,12},{-44,28}})));
        ProsNet.Fluid.Building_Fluid.Sensors.EnthalpyFlowRate HOutEnh_flow(
            redeclare package Medium = Medium, m_flow_nominal=m_flow_nominal)
          "Enthalpy flow rate"
          annotation (Placement(transformation(extent={{2,12},{18,28}})));
        Modelica.Blocks.Math.Add add(k2=-1) "Adder for enthalpy difference"
          annotation (Placement(transformation(extent={{20,40},{40,60}})));
        Modelica.Blocks.Continuous.Integrator dHTanEnh
          "Difference in enthalpy (should be zero at steady-state)"
          annotation (Placement(transformation(extent={{60,40},{80,60}})));
        Modelica.Blocks.Math.Add add1(k2=-1) "Adder for enthalpy difference"
          annotation (Placement(transformation(extent={{20,-30},{40,-10}})));
        Modelica.Blocks.Continuous.Integrator dHTan
          "Difference in enthalpy (should be zero at steady-state)"
          annotation (Placement(transformation(extent={{60,-30},{80,-10}})));

      equation
        connect(HIn_flow.port_b, cooTan.port_a) annotation (Line(
            points={{-42,-50},{-30,-50}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(cooTan.port_b, HOut_flow.port_a) annotation (Line(
            points={{-10,-50},{2,-50}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(HOut_flow.port_b, sin_2.ports[1]) annotation (Line(
            points={{18,-50},{60,-50}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(HOutEnh_flow.port_b, sin_1.ports[1]) annotation (Line(
            points={{18,20},{60,20}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(HInEnh_flow.H_flow, add.u1) annotation (Line(
            points={{-52,28.8},{-52,56},{18,56}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(HOutEnh_flow.H_flow, add.u2) annotation (Line(
            points={{10,28.8},{10,44},{18,44}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(add.y, dHTanEnh.u) annotation (Line(
            points={{41,50},{58,50}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(HIn_flow.H_flow, add1.u1) annotation (Line(
            points={{-50,-41.2},{-50,-14},{18,-14}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(HOut_flow.H_flow, add1.u2) annotation (Line(
            points={{10,-41.2},{10,-26},{18,-26}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(add1.y, dHTan.u) annotation (Line(
            points={{41,-20},{58,-20}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(HInEnh_flow.port_b, heaTan.port_a)
          annotation (Line(points={{-44,20},{-30,20}}, color={0,127,255}));
        connect(heaTan.port_b, HOutEnh_flow.port_a)
          annotation (Line(points={{-10,20},{2,20}}, color={0,127,255}));
        connect(sou_1.ports[1], HInEnh_flow.port_a) annotation (Line(points={{-80,-8},
                {-70,-8},{-70,20},{-60,20}}, color={0,127,255}));
        connect(sou_1.ports[2], HIn_flow.port_a) annotation (Line(points={{-80,-12},{-70,
                -12},{-70,-50},{-58,-50}}, color={0,127,255}));

      annotation (experiment(Tolerance=1e-6, StopTime=3600),
        __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Storage/Validation/StratifiedNonUniformInitial.mos"
              "Simulate and plot"),
      Documentation(info="<html>
This test model validates
<a href=\"modelica://Buildings.Fluid.Storage.Stratified\">
Buildings.Fluid.Storage.Stratified</a> by specifying a non-uniform initial 
temperature. 
</html>",       revisions="<html>
<ul>
<li>
November 13, 2019 by Jianjun Hu:<br/>
Changed the uniform initial tank temperature to be non-uniform.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1246\">#1246</a>.
</li>
</ul>
</html>"));
      end StratifiedNonUniformInitial;
    annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains models that validate the storage models.
These model outputs are stored as reference data to
allow continuous validation whenever models in the library change.
</p>
</html>"));
    end Validation;

    package BaseClasses "Package with base classes for Buildings.Fluid.Storage"
      extends Modelica.Icons.BasesPackage;

      model Buoyancy
        "Model to add buoyancy if there is a temperature inversion in the tank"
        extends Modelica.Blocks.Icons.Block;

        replaceable package Medium = Modelica.Media.Interfaces.PartialMedium "Medium model"  annotation (
            choicesAllMatching = true);
        parameter Modelica.Units.SI.Volume V "Volume";
        parameter Integer nSeg(min=2) = 2 "Number of volume segments";
        parameter Modelica.Units.SI.Time tau(min=0) "Time constant for mixing";

        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[nSeg] heatPort
          "Heat input into the volumes"
          annotation (Placement(transformation(extent={{90,-10},{110,10}})));

        Modelica.Units.SI.HeatFlowRate[nSeg - 1] Q_flow
          "Heat flow rate from segment i+1 to i";
      protected
         parameter Medium.ThermodynamicState sta_default = Medium.setState_pTX(T=Medium.T_default,
               p=Medium.p_default, X=Medium.X_default[1:Medium.nXi])
          "Medium state at default properties";
        parameter Modelica.Units.SI.Density rho_default=Medium.density(sta_default)
          "Density, used to compute fluid mass";
        parameter Modelica.Units.SI.SpecificHeatCapacity cp_default=
            Medium.specificHeatCapacityCp(sta_default) "Specific heat capacity";
         parameter Real k(unit="W/K") = V*rho_default*cp_default/tau/nSeg
          "Proportionality constant, since we use dT instead of dH";
        Modelica.Units.SI.TemperatureDifference dT[nSeg - 1]
          "Temperature difference between adjoining volumes";
      equation
        for i in 1:nSeg-1 loop
          dT[i] = heatPort[i+1].T-heatPort[i].T;
          Q_flow[i] = k*noEvent(smooth(1, if dT[i]>0 then dT[i]^2 else 0));
        end for;

        heatPort[1].Q_flow = -Q_flow[1];
        for i in 2:nSeg-1 loop
             heatPort[i].Q_flow = -Q_flow[i]+Q_flow[i-1];
        end for;
        heatPort[nSeg].Q_flow = Q_flow[nSeg-1];
        annotation (Documentation(info="<html>
<p>
This model outputs a heat flow rate that can be added to fluid volumes
in order to emulate buoyancy during a temperature inversion.
For simplicity, this model does not compute a buoyancy induced mass flow rate,
but rather a heat flow that has the same magnitude as the enthalpy flow
associated with the buoyancy induced mass flow rate.
</p>
</html>",       revisions="<html>
<ul>
<li>
June 7, 2018 by Filip Jorissen:<br/>
Copied model from Buildings and update the model accordingly.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/314\">#314</a>.
</li>
<li>
December 14, 2012 by Michael Wetter:<br/>
Renamed protected parameters for consistency with naming convention.
</li>
<li>
October 8, 2011 by Michael Wetter:<br/>
Added <code>noEvent(...)</code> to
<code>Q_flow[i] = k*smooth(1, if dT[i]>0 then dT[i]^2 else 0);</code>
since the equation returns the same value to the left and right of
<code>dT[i]>0</code>.
</li>
<li>
September 16, 2011 by Michael Wetter:<br/>
Changed the implementation from <code>Q_flow[i] = k*max(heatPort[i+1].T-heatPort[i].T, 0);</code> to
<code>Q_flow[i] = k*smooth(1, if dT[i]>0 then dT[i]^2 else 0);</code>.
The previous implementation was not differentiable. In modeling a solar system, this
change reduced the computing time by a factor of 20 during the time when the pumps
were almost switched off and colder temperature was fed from the collector to the tank.
</li>
<li>
October 28, 2008 by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
              graphics={
              Rectangle(
                extent={{-44,68},{36,28}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={0,0,127},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-42,-26},{38,-66}},
                lineColor={255,0,0},
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{26,10},{32,-22}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{28,22},{22,10},{36,10},{36,10},{28,22}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-32,22},{-26,-10}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={0,0,127},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-28,-18},{-36,-6},{-22,-6},{-22,-6},{-28,-18}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={0,0,127},
                fillPattern=FillPattern.Solid)}));
      end Buoyancy;

      model IndirectTankHeatExchanger
        "Heat exchanger typically submerged in a fluid with a second fluid circulating through it"

        replaceable package MediumHex = Modelica.Media.Interfaces.PartialMedium
          "Heat transfer fluid flowing through the heat exchanger";
        replaceable package MediumTan = Modelica.Media.Interfaces.PartialMedium
          "Heat transfer fluid inside the tank";

        extends
          ProsNet.Fluid.Building_Fluid.Interfaces.TwoPortFlowResistanceParameters;
        extends
          ProsNet.Fluid.Building_Fluid.Interfaces.LumpedVolumeDeclarations(
            final massDynamics=energyDynamics, redeclare final package Medium =
              MediumHex);
        extends ProsNet.Fluid.Building_Fluid.Interfaces.PartialTwoPortInterface(
            redeclare final package Medium = MediumHex, final show_T=false);

        constant Boolean homotopyInitialization = true "= true, use homotopy method"
          annotation(HideResult=true);

        parameter Integer nSeg(min=2) "Number of segments in the heat exchanger";
        parameter Modelica.Units.SI.HeatCapacity CHex
          "Capacitance of the heat exchanger";
        parameter Modelica.Units.SI.Volume volHexFlu
          "Volume of heat transfer fluid in the heat exchanger";
        parameter Modelica.Units.SI.HeatFlowRate Q_flow_nominal
          "Heat transfer at nominal conditions"
          annotation (Dialog(tab="General", group="Nominal condition"));

        final parameter Modelica.Units.SI.ThermalConductance UA_nominal=abs(
            Q_flow_nominal/(THex_nominal - TTan_nominal))
          "Nominal UA value for the heat exchanger";
        parameter Modelica.Units.SI.Temperature TTan_nominal
          "Temperature of fluid inside the tank at UA_nominal"
          annotation (Dialog(tab="General", group="Nominal condition"));
        parameter Modelica.Units.SI.Temperature THex_nominal
          "Temperature of fluid inside the heat exchanger at UA_nominal"
          annotation (Dialog(tab="General", group="Nominal condition"));
        parameter Real r_nominal(min=0, max=1)=0.5
          "Ratio between coil inside and outside convective heat transfer"
                annotation(Dialog(tab="General", group="Nominal condition"));

        parameter Modelica.Units.SI.Diameter dExtHex
          "Exterior diameter of the heat exchanger pipe";

        parameter Modelica.Fluid.Types.Dynamics energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial
          "Formulation of energy balance for heat exchanger internal fluid mass"
          annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Conservation equations"));
        parameter Modelica.Fluid.Types.Dynamics energyDynamicsSolid=energyDynamics
          "Formulation of energy balance for heat exchanger solid mass"
          annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Conservation equations"));

        parameter Boolean hA_flowDependent = true
          "Set to false to make the convective heat coefficient calculation of the fluid inside the coil independent of mass flow rate"
          annotation(Dialog(tab="Advanced", group="Modeling detail"), Evaluate=true);
        parameter Boolean hA_temperatureDependent = true
          "Set to false to make the convective heat coefficient calculation of the fluid inside the coil independent of temperature"
          annotation(Dialog(tab="Advanced", group="Modeling detail"), Evaluate=true);

        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port[nSeg]
          "Heat port connected to water inside the tank"
          annotation (Placement(transformation(extent={{-10,-160},{10,-140}}),
              iconTransformation(extent={{-10,-108},{10,-88}})));

        ProsNet.Fluid.Building_Fluid.FixedResistances.PressureDrop res(
          redeclare final package Medium = MediumHex,
          final dp_nominal=dp_nominal,
          final m_flow_nominal=m_flow_nominal,
          final allowFlowReversal=allowFlowReversal,
          final homotopyInitialization=homotopyInitialization,
          final show_T=show_T,
          final from_dp=from_dp,
          final linearized=linearizeFlowResistance)
          "Calculates the flow resistance and pressure drop through the heat exchanger"
          annotation (Placement(transformation(extent={{46,-60},{66,-40}})));

        ProsNet.Fluid.Building_Fluid.MixingVolumes.MixingVolume vol[nSeg](
          redeclare each package Medium = MediumHex,
          each nPorts=2,
          each m_flow_nominal=m_flow_nominal,
          each V=volHexFlu/nSeg,
          each energyDynamics=energyDynamics,
          each massDynamics=energyDynamics,
          each p_start=p_start,
          each T_start=T_start,
          each X_start=X_start,
          each C_start=C_start,
          each C_nominal=C_nominal,
          each final prescribedHeatFlowRate=false,
          each final allowFlowReversal=allowFlowReversal)
          "Heat exchanger fluid"
          annotation (Placement(transformation(extent={{-32,-40},{-12,-20}})));
        Modelica.Thermal.HeatTransfer.Components.HeatCapacitor cap[nSeg](
           each C=CHex/nSeg,
           each T(start=T_start,
                  fixed=(energyDynamicsSolid == Modelica.Fluid.Types.Dynamics.FixedInitial)),
           each der_T(
                  fixed=(energyDynamicsSolid == Modelica.Fluid.Types.Dynamics.SteadyStateInitial)))
                if not energyDynamicsSolid == Modelica.Fluid.Types.Dynamics.SteadyState
          "Thermal mass of the heat exchanger"
          annotation (Placement(transformation(extent={{-6,6},{14,26}})));
      protected
        ProsNet.Fluid.Building_Fluid.Sensors.MassFlowRate senMasFlo(redeclare
            package Medium = MediumHex, allowFlowReversal=allowFlowReversal)
          "Mass flow rate of the heat transfer fluid"
          annotation (Placement(transformation(extent={{-80,-40},{-60,-60}})));
        Modelica.Thermal.HeatTransfer.Components.Convection htfToHex[nSeg]
          "Convection coefficient between the heat transfer fluid and heat exchanger"
          annotation (Placement(transformation(extent={{-10,12},{-30,-8}})));
        Modelica.Thermal.HeatTransfer.Components.Convection HexToTan[nSeg]
          "Convection coefficient between the heat exchanger and the surrounding medium"
          annotation (Placement(transformation(extent={{20,12},{40,-8}})));
        Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temSenHex[nSeg]
          "Temperature of the heat transfer fluid"
          annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              origin={-20,-70})));
        Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temSenWat[nSeg]
          "Temperature sensor of the fluid surrounding the heat exchanger"
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={68,40})));
        Modelica.Blocks.Routing.Replicator rep(nout=nSeg)
          "Replicates senMasFlo signal from 1 seg to nSeg"
          annotation (Placement(transformation(extent={{-44,-108},{-24,-88}})));
        ProsNet.Fluid.Building_Fluid.HeatExchangers.BaseClasses.HACoilInside
          hAPipIns[nSeg](
          each m_flow_nominal=m_flow_nominal,
          each hA_nominal=UA_nominal/nSeg*(r_nominal + 1)/r_nominal,
          each T_nominal=THex_nominal,
          each final flowDependent=hA_flowDependent,
          each final temperatureDependent=hA_temperatureDependent)
          "Computation of convection coefficients inside the coil" annotation (
            Placement(transformation(extent={{-10,-10},{10,10}}, origin={20,-80})));
        ProsNet.Fluid.Building_Fluid.HeatExchangers.BaseClasses.HANaturalCylinder
          hANatCyl[nSeg](
          redeclare each final package Medium = Medium,
          each final ChaLen=dExtHex,
          each final hA_nominal=UA_nominal/nSeg*(1 + r_nominal),
          each final TFlu_nominal=TTan_nominal,
          each final TSur_nominal=TTan_nominal - (r_nominal/(1 + r_nominal))*(
              TTan_nominal - THex_nominal))
          "Calculates an hA value for each side of the heat exchanger"
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
                origin={10,110})));
        Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temSenSur[nSeg]
          "Temperature at the external surface of the heat exchanger"
          annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={20,42})));

      initial equation
        assert(homotopyInitialization, "In " + getInstanceName() +
          ": The constant homotopyInitialization has been modified from its default value. This constant will be removed in future releases.",
          level = AssertionLevel.warning);

      equation
        for i in 1:(nSeg - 1) loop
          connect(vol[i].ports[2], vol[i + 1].ports[1]);
        end for;

        connect(rep.u,senMasFlo. m_flow) annotation (Line(
            points={{-46,-98},{-70,-98},{-70,-61}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(port,HexToTan. fluid)    annotation (Line(
            points={{4.44089e-16,-150},{88,-150},{88,2},{40,2}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(vol[1].ports[1],senMasFlo.port_b) annotation (Line(
            points={{-23,-40},{-23,-50},{-60,-50}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(cap.port,HexToTan.solid) annotation (Line(
            points={{4,6},{4,2},{20,2}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(vol.heatPort,htfToHex. fluid) annotation (Line(
            points={{-32,-30},{-36,-30},{-36,2},{-30,2}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(htfToHex.solid,HexToTan. solid) annotation (Line(
            points={{-10,2},{20,2}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(temSenHex.T, hAPipIns.T)     annotation (Line(
            points={{-9,-70},{0,-70},{0,-76},{9,-76}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(hAPipIns.hA, htfToHex.Gc)     annotation (Line(
            points={{31,-80},{32,-80},{32,-18},{-20,-18},{-20,-8}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(HexToTan.solid,temSenSur. port) annotation (Line(
            points={{20,2},{20,32}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(temSenWat.port, port)    annotation (Line(
            points={{68,30},{68,2},{88,2},{88,-150},{4.44089e-16,-150}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(temSenSur.T, hANatCyl.TSur)
                                           annotation (Line(
            points={{20,53},{20,70},{-40,70},{-40,114},{-2,114}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(hANatCyl.TFlu, temSenWat.T)
                                           annotation (Line(
            points={{-2,106},{-36,106},{-36,76},{68,76},{68,51}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(port_a, senMasFlo.port_a) annotation (Line(
            points={{-100,0},{-90,0},{-90,-50},{-80,-50}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(vol[nSeg].ports[2], res.port_a) annotation (Line(
            points={{-21,-40},{-21,-50},{46,-50}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(res.port_b, port_b) annotation (Line(
            points={{66,-50},{84,-50},{84,4.44089e-16},{100,4.44089e-16}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(temSenHex.port, vol.heatPort) annotation (Line(
            points={{-30,-70},{-36,-70},{-36,-30},{-32,-30}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(rep.y, hAPipIns.m_flow)     annotation (Line(
            points={{-23,-98},{0,-98},{0,-84},{9,-84}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(hANatCyl.hA,HexToTan. Gc) annotation (Line(
            points={{21,110},{50,110},{50,-14},{30,-14},{30,-8}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -150},{100,150}}), graphics), Icon(coordinateSystem(
                preserveAspectRatio=false, extent={{-100,-150},{100,150}}), graphics={
              Rectangle(
                extent={{-70,64},{70,-96}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-100,5},{101,-5}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={0,0,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-70,-12},{70,-18}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-40,64},{-36,-96}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-2,64},{2,-96}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{36,64},{40,-96}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid)}),
                defaultComponentName="indTanHex",
              Documentation(info = "<html>
          <p>
          This model is a heat exchanger with a moving fluid on one side and a stagnant fluid on the other.
          It is intended for use when a heat exchanger is submerged in a stagnant fluid.
          For example, the heat exchanger in a storage tank which is part of a solar thermal system.
          </p>
          <p>
          This component models the fluid in the heat exchanger, convection between the fluid and
          the heat exchanger, and convection from the heat exchanger to the surrounding fluid.
          </p>
          <p>
          The model is based on <a href=\"modelica://Buildings.Fluid.HeatExchangers.BaseClasses.HACoilInside\">
          Buildings.Fluid.HeatExchangers.BaseClasses.HACoilInside</a> and
          <a href=\"modelica://Buildings.Fluid.HeatExchangers.BaseClasses.HANaturalCylinder\">
          Buildings.Fluid.HeatExchangers.BaseClasses.HANaturalCylinder</a>.
          </p>
          <p>
          The fluid ports are intended to be connected to a circulated heat transfer fluid
          while the heat port is intended to be connected to a stagnant fluid.
          </p>
          </html>",
                revisions="<html>
<ul>
<li>
March 7, 2022, by Michael Wetter:<br/>
Set <code>final massDynamics=energyDynamics</code>.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1542\">#1542</a>.
</li>
<li>
April 9, 2021, by Michael Wetter:<br/>
Corrected placement of <code>each</code> keyword.<br/>
See <a href=\"https://github.com/lbl-srg/modelica-buildings/pull/2440\">Buildings, PR #2440</a>.
</li>
<li>
April 14, 2020, by Michael Wetter:<br/>
Changed <code>homotopyInitialization</code> to a constant.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1341\">IBPSA, #1341</a>.
</li>
<li>
June 7, 2018 by Filip Jorissen:<br/>
Copied model from Buildings and update the model accordingly.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/314\">#314</a>.
</li>
<li>
January 7, 2016, by Filip Jorissen:<br/>
Propagated <code>flowDependent</code> and <code>temperatureDependent</code>
in <code>hAPipIns</code>.
This is for issue
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/454\">#454</a>.
</li>
<li>
September 24, 2015 by Michael Wetter:<br/>
Set <code>fixed</code> attribute in <code>cap.T</code> to avoid
unspecified initial conditions.
</li>
<li>
July 2, 2015, by Michael Wetter:<br/>
Set <code>prescribedHeatFlowRate=false</code> in control volume.
</li>
<li>
July 1, 2015, by Filip Jorissen:<br/>
Added parameter <code>energyDynamicsSolid</code>.
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/434\">
#434</a>.
</li>
<li>
March 28, 2015, by Filip Jorissen:<br/>
Propagated <code>allowFlowReversal</code>.
</li>
          <li>
          August 29, 2014, by Michael Wetter:<br/>
          Introduced <code>MediumTan</code> for the tank medium, and assigned <code>Medium</code>
          to be equal to <code>MediumHex</code>.
          This is to correct issue <a href=\"https://github.com/lbl-srg/modelica-buildings/issues/271\">
          #271</a>.
          </li>
          <li>
          June 18, 2014, by Michael Wetter:<br/>
          Set initial equations for <code>cap</code>, and renamed this instance from
          <code>Cap</code> to <code>cap</code>.
          This was done to avoid a warning during translation, and to comply with
          the coding convention.
          </li>
          <li>
          October 8, 2013, by Michael Wetter:<br/>
          Removed parameter <code>show_V_flow</code>.
          </li>
          <li>
          January 29, 2013, by Peter Grant:<br/>
          First implementation.
          </li>
          </ul>
          </html>"));
      end IndirectTankHeatExchanger;

      model PartialStratified
        "Partial model of a stratified tank for thermal energy storage"
        extends
          ProsNet.Under_Development.Storage.Storage.BaseClasses.PartialTwoPortInterface;

        import Modelica.Fluid.Types;
        import Modelica.Fluid.Types.Dynamics;

        parameter Modelica.Units.SI.Volume VTan "Tank volume";
        parameter Modelica.Units.SI.Length hTan "Height of tank (without insulation)";
        parameter Modelica.Units.SI.Length dIns "Thickness of insulation";
        parameter Modelica.Units.SI.ThermalConductivity kIns=0.04
          "Specific heat conductivity of insulation";
        parameter Integer nSeg(min=2) = 2 "Number of volume segments";

        ////////////////////////////////////////////////////////////////////
        // Assumptions
        parameter Types.Dynamics energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial
          "Formulation of energy balance"
          annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Conservation equations"));

        // Initialization
        parameter Medium.AbsolutePressure p_start = Medium.p_default
          "Start value of pressure"
          annotation(Dialog(tab = "Initialization"));
        parameter Medium.Temperature T_start=Medium.T_default
          "Start value of temperature"
          annotation(Dialog(tab = "Initialization"));
        parameter Modelica.Units.SI.Temperature TFlu_start[nSeg]=T_start*ones(nSeg)
          "Initial temperature of the tank segments, with TFlu_start[1] being the top segment"
          annotation (Dialog(tab="Initialization"));
        parameter Medium.MassFraction X_start[Medium.nX] = Medium.X_default
          "Start value of mass fractions m_i/m"
          annotation (Dialog(tab="Initialization", enable=Medium.nXi > 0));
        parameter Medium.ExtraProperty C_start[Medium.nC](
             quantity=Medium.extraPropertiesNames)=fill(0, Medium.nC)
          "Start value of trace substances"
          annotation (Dialog(tab="Initialization", enable=Medium.nC > 0));

        // Dynamics
        parameter Modelica.Units.SI.Time tau=1 "Time constant for mixing";

        ////////////////////////////////////////////////////////////////////
        // Connectors

        Modelica.Blocks.Interfaces.RealOutput Ql_flow
          "Heat loss of tank (positive if heat flows from tank to ambient)"
          annotation (Placement(transformation(extent={{100,62},{120,82}})));

        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[nSeg] heaPorVol
          "Heat port that connects to the control volumes of the tank"
          annotation (Placement(transformation(extent={{-6,-6},{6,6}})));

        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heaPorSid
          "Heat port tank side (outside insulation)"
          annotation (Placement(transformation(extent={{50,-6},{62,6}})));

        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heaPorTop
          "Heat port tank top (outside insulation)"
          annotation (Placement(transformation(extent={{14,68},{26,80}})));
        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heaPorBot
          "Heat port tank bottom (outside insulation). Leave unconnected for adiabatic condition"
          annotation (Placement(transformation(extent={{14,-80},{26,-68}})));

        // Models
        ProsNet.Fluid.Building_Fluid.MixingVolumes.MixingVolume[nSeg] vol(
          redeclare each package Medium = Medium,
          each energyDynamics=energyDynamics,
          each massDynamics=energyDynamics,
          each p_start=p_start,
          T_start=TFlu_start,
          each X_start=X_start,
          each C_start=C_start,
          each V=VTan/nSeg,
          each m_flow_nominal=m_flow_nominal,
          each final mSenFac=1,
          each final m_flow_small=m_flow_small,
          each final allowFlowReversal=allowFlowReversal) "Tank segment"
          annotation (Placement(transformation(extent={{6,-16},{26,4}})));

      protected
        parameter Medium.ThermodynamicState sta_default = Medium.setState_pTX(
          T=Medium.T_default,
          p=Medium.p_default,
          X=Medium.X_default[1:Medium.nXi]) "Medium state at default properties";
        parameter Modelica.Units.SI.Length hSeg=hTan/nSeg "Height of a tank segment";
        parameter Modelica.Units.SI.Area ATan=VTan/hTan
          "Tank cross-sectional area (without insulation)";
        parameter Modelica.Units.SI.Length rTan=sqrt(ATan/Modelica.Constants.pi)
          "Tank diameter (without insulation)";
        parameter Modelica.Units.SI.ThermalConductance conFluSeg=ATan*
            Medium.thermalConductivity(sta_default)/hSeg
          "Thermal conductance between fluid volumes";
        parameter Modelica.Units.SI.ThermalConductance conTopSeg=ATan*kIns/dIns
          "Thermal conductance from center of top (or bottom) volume through tank insulation at top (or bottom)";

        BaseClasses.Buoyancy buo(
          redeclare final package Medium = Medium,
          final V=VTan,
          final nSeg=nSeg,
          final tau=tau) "Model to prevent unstable tank stratification"
          annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
        Modelica.Thermal.HeatTransfer.Components.ThermalConductor[nSeg - 1] conFlu(
          each G=conFluSeg) "Thermal conductance in fluid between the segments"
          annotation (Placement(transformation(extent={{-56,4},{-42,18}})));
        Modelica.Thermal.HeatTransfer.Components.ThermalConductor[nSeg] conWal(
           each G=2*Modelica.Constants.pi*kIns*hSeg/Modelica.Math.log((rTan+dIns)/rTan))
          "Thermal conductance through tank wall"
          annotation (Placement(transformation(extent={{10,34},{20,46}})));
        Modelica.Thermal.HeatTransfer.Components.ThermalConductor conTop(
           G=conTopSeg) "Thermal conductance through tank top"
          annotation (Placement(transformation(extent={{10,54},{20,66}})));
        Modelica.Thermal.HeatTransfer.Components.ThermalConductor conBot(
           G=conTopSeg) "Thermal conductance through tank bottom"
          annotation (Placement(transformation(extent={{10,14},{20,26}})));

        Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor heaFloTop
          "Heat flow at top of tank (outside insulation)"
          annotation (Placement(transformation(extent={{30,54},{42,66}})));
        Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor heaFloBot
          "Heat flow at bottom of tank (outside insulation)"
          annotation (Placement(transformation(extent={{30,14},{42,26}})));
        Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor heaFloSid[nSeg]
          "Heat flow at wall of tank (outside insulation)"
          annotation (Placement(transformation(extent={{30,34},{42,46}})));

        Modelica.Blocks.Routing.Multiplex3 mul(
          n1=1,
          n2=nSeg,
          n3=1) "Multiplex to collect heat flow rates"
          annotation (Placement(transformation(extent={{62,44},{70,54}})));
        Modelica.Blocks.Math.Sum sum1(nin=nSeg + 2)
        annotation (Placement(transformation(extent={{78,42},{90,56}})));

        Modelica.Thermal.HeatTransfer.Components.ThermalCollector theCol(m=nSeg)
          "Connector to assign multiple heat ports to one heat port"
          annotation (Placement(transformation(extent={{46,20},{58,32}})));
      equation
        connect(buo.heatPort, vol.heatPort)    annotation (Line(
            points={{-40,60},{6,60},{6,-6}},
            color={191,0,0}));
        for i in 1:nSeg-1 loop
        // heat conduction between fluid nodes
           connect(vol[i].heatPort, conFlu[i].port_a)    annotation (Line(points={{6,-6},{
                  6,-6},{-60,-6},{-60,10},{-56,10},{-56,11}},    color={191,0,0}));
          connect(vol[i+1].heatPort, conFlu[i].port_b)    annotation (Line(points={{6,-6},{
                  -40,-6},{-40,11},{-42,11}},  color={191,0,0}));
        end for;
        connect(vol[1].heatPort, conTop.port_a)    annotation (Line(points={{6,-6},{6,
                60},{-4,60},{10,60}},              color={191,0,0}));
        connect(vol.heatPort, conWal.port_a)    annotation (Line(points={{6,-6},{6,40},
                {10,40}},                      color={191,0,0}));
        connect(conBot.port_a, vol[nSeg].heatPort)    annotation (Line(points={{10,20},
                {10,20},{6,20},{6,-6}},
                                     color={191,0,0}));
        connect(vol.heatPort, heaPorVol)    annotation (Line(points={{6,-6},{6,-6},{
                -2.22045e-16,-6},{-2.22045e-16,-2.22045e-16}},
              color={191,0,0}));
        connect(conWal.port_b, heaFloSid.port_a)
          annotation (Line(points={{20,40},{30,40}}, color={191,0,0}));

        connect(conTop.port_b, heaFloTop.port_a)
          annotation (Line(points={{20,60},{30,60}}, color={191,0,0}));
        connect(conBot.port_b, heaFloBot.port_a)
          annotation (Line(points={{20,20},{30,20}}, color={191,0,0}));
        connect(heaFloTop.port_b, heaPorTop) annotation (Line(points={{42,60},{52,60},
                {52,74},{20,74}}, color={191,0,0}));
        connect(heaFloBot.port_b, heaPorBot) annotation (Line(points={{42,20},{44,20},
                {44,-74},{20,-74}}, color={191,0,0}));
        connect(heaFloTop.Q_flow, mul.u1[1]) annotation (Line(points={{36,53.4},{50,53.4},
                {50,52.5},{61.2,52.5}}, color={0,0,127}));
        connect(heaFloSid.Q_flow, mul.u2) annotation (Line(points={{36,33.4},{50,33.4},
                {50,49},{61.2,49}},color={0,0,127}));
        connect(heaFloBot.Q_flow, mul.u3[1]) annotation (Line(points={{36,13.4},{36,10},
                {58,10},{58,45.5},{61.2,45.5}}, color={0,0,127}));
        connect(mul.y, sum1.u) annotation (Line(points={{70.4,49},{76.8,49}}, color={
                0,0,127}));
        connect(sum1.y, Ql_flow) annotation (Line(points={{90.6,49},{98,49},{98,72},{
                110,72}}, color={0,0,127}));
        connect(heaFloSid.port_b, theCol.port_a) annotation (Line(
            points={{42,40},{52,40},{52,32}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(theCol.port_b, heaPorSid) annotation (Line(
            points={{52,20},{52,-2.22045e-16},{56,-2.22045e-16}},
            color={191,0,0},
            smooth=Smooth.None));
        annotation (
      Documentation(info="<html>
<p>
This is a partial model of a stratified storage tank.
</p>
<p>
See the
<a href=\"modelica://Buildings.Fluid.Storage.UsersGuide\">
Buildings.Fluid.Storage.UsersGuide</a>
for more information.
</p>
<h4>Implementation</h4>
<p>
This model does not include the ports that connect to the fluid from
the outside, because these ports cannot be used for the models that
contain the
<a href=\"modelica://Buildings.Fluid.Storage.BaseClasses.ThirdOrderStratifier\">
Buildings.Fluid.Storage.BaseClasses.ThirdOrderStratifier</a>.
</p>
</html>",       revisions="<html>
<ul>
<li>
March 7, 2022, by Michael Wetter:<br/>
Set <code>final massDynamics=energyDynamics</code>.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1542\">#1542</a>.
</li>
<li>
November 13, 2019 by Jianjun Hu:<br/>
Added parameter <code>TFlu_start</code> and changed the initial tank segments
temperature to <code>TFlu_start</code> so each segment could have different
temperature.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1246\">#1246</a>.
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
                extent={{2,100},{-2,60}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={0,0,127},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{2,-60},{-2,-100}},
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
                pattern=LinePattern.Dot),     Text(
              extent={{-100,100},{-8,70}},
              textString="%name",
              textColor={0,0,255})}));
      end PartialStratified;

      partial model PartialTwoPortInterface
        "Partial model transporting fluid between two ports without storing mass or energy"

        replaceable package Medium =
          Modelica.Media.Interfaces.PartialMedium "Medium in the component"
            annotation (choices(
              choice(redeclare package Medium = Buildings.Media.Air "Moist air"),
              choice(redeclare package Medium = Buildings.Media.Water "Water"),
              choice(redeclare package Medium =
                  Buildings.Media.Antifreeze.PropyleneGlycolWater (
                    property_T=293.15,
                    X_a=0.40)
                    "Propylene glycol water, 40% mass fraction")));

        parameter Boolean allowFlowReversal = true
          "= false to simplify equations, assuming, but not enforcing, no flow reversal"
          annotation(Dialog(tab="Assumptions"), Evaluate=true);

        Modelica.Fluid.Interfaces.FluidPort_a port_a(
          redeclare final package Medium = Medium,
          m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0),
          p(start=Medium.p_default),
          h_outflow(start = Medium.h_default, nominal = Medium.h_default))
          "Fluid connector a (positive design flow direction is from port_a to port_b)"
          annotation (Placement(transformation(extent={{-10,90},{10,110}})));
        Modelica.Fluid.Interfaces.FluidPort_b port_b(
          redeclare final package Medium = Medium,
          m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0),
          p(start=Medium.p_default),
          h_outflow(start = Medium.h_default, nominal = Medium.h_default))
          "Fluid connector b (positive design flow direction is from port_a to port_b)"
          annotation (Placement(transformation(extent={{10,-110},{-10,-90}})));

        parameter Modelica.Units.SI.MassFlowRate m_flow_nominal
          "Nominal mass flow rate" annotation (Dialog(group="Nominal condition"));
        parameter Modelica.Units.SI.MassFlowRate m_flow_small(min=0) = 1E-4*abs(
          m_flow_nominal) "Small mass flow rate for regularization of zero flow"
          annotation (Dialog(tab="Advanced"));
        // Diagnostics
         parameter Boolean show_T = false
          "= true, if actual temperature at port is computed"
          annotation (
            Dialog(tab="Advanced", group="Diagnostics"),
            HideResult=true);

        Modelica.Units.SI.MassFlowRate m_flow(start=_m_flow_start) = port_a.m_flow
          "Mass flow rate from port_a to port_b (m_flow > 0 is design flow direction)";

        Modelica.Units.SI.PressureDifference dp(
          start=_dp_start,
          displayUnit="Pa") = port_a.p - port_b.p
          "Pressure difference between port_a and port_b";

        Medium.ThermodynamicState sta_a=
          if allowFlowReversal then
            Medium.setState_phX(port_a.p,
                                noEvent(actualStream(port_a.h_outflow)),
                                noEvent(actualStream(port_a.Xi_outflow)))
          else
            Medium.setState_phX(port_a.p,
                                noEvent(inStream(port_a.h_outflow)),
                                noEvent(inStream(port_a.Xi_outflow)))
            if show_T "Medium properties in port_a";

        Medium.ThermodynamicState sta_b=
          if allowFlowReversal then
            Medium.setState_phX(port_b.p,
                                noEvent(actualStream(port_b.h_outflow)),
                                noEvent(actualStream(port_b.Xi_outflow)))
          else
            Medium.setState_phX(port_b.p,
                                noEvent(port_b.h_outflow),
                                noEvent(port_b.Xi_outflow))
             if show_T "Medium properties in port_b";

      protected
        final parameter Modelica.Units.SI.MassFlowRate _m_flow_start=0
          "Start value for m_flow, used to avoid a warning if not set in m_flow, and to avoid m_flow.start in parameter window";
        final parameter Modelica.Units.SI.PressureDifference _dp_start(displayUnit=
              "Pa") = 0
          "Start value for dp, used to avoid a warning if not set in dp, and to avoid dp.start in parameter window";

        annotation (
          preferredView="info",
          Documentation(info="<html>
<p>
This partial class implements the same functionality as 
<a href=\"modelica://Buildings.Fluid.Interfaces.PartialTwoPortInterface\">
Buildings.Fluid.Interfaces.PartialTwoPortInterface</a>,
except that <code>port_a</code> and <code>port_b</code> are placed at the top and bottom
of the component.
</p>
<h4>Implementation</h4>
<p>
The implementation is done in this package as opposed to
<a href=\"modelica://Buildings.Fluid.Interfaces\">
Buildings.Fluid.Interfaces</a>
as it is only used by the storage model, and may be removed when the tool limitations
that are discussed in
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1794\">IBPSA, #1794</a>.
are removed.
</p>
</html>",       revisions="<html>
<ul>
<li>
September 20, 2023, by Michael Wetter:<br/>
First implementation to address
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1794\">IBPSA, #1794</a>.
</li>
</ul>
</html>"));
      end PartialTwoPortInterface;

      model ThirdOrderStratifier
        "Model to reduce the numerical dissipation in a tank"
        extends Modelica.Blocks.Icons.Block;

        replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
          "Medium model" annotation (choicesAllMatching=true);

        parameter Modelica.Units.SI.MassFlowRate m_flow_small(min=0)
          "Small mass flow rate for regularization of zero flow";
        parameter Integer nSeg(min=4) "Number of volume segments";

        parameter Real alpha(
          min=0,
          max=1) = 0.5 "Under-relaxation coefficient (1: QUICK; 0: 1st order upwind)";

        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[nSeg] heatPort
          "Heat input into the volumes" annotation (Placement(transformation(extent={
                  {90,-10},{110,10}})));

        Modelica.Blocks.Interfaces.RealInput m_flow
          "Mass flow rate from port a to port b" annotation (Placement(transformation(
                extent={{-140,62},{-100,102}})));

        Modelica.Blocks.Interfaces.RealInput[nSeg + 1] H_flow
          "Enthalpy flow between the volumes" annotation (Placement(transformation(
                extent={{-140,-100},{-100,-60}})));

        Modelica.Fluid.Interfaces.FluidPort_a[nSeg + 2] fluidPort(redeclare
            each package Medium =
                             Medium)
          "Fluid port, needed to get pressure, temperature and species concentration"
          annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));

      protected
        Modelica.Units.SI.SpecificEnthalpy[nSeg + 1] hOut
          "Extended vector with new outlet enthalpies to reduce numerical dissipation (at the boundary between two volumes)";
        Modelica.Units.SI.SpecificEnthalpy[nSeg + 2] h
          "Extended vector with port enthalpies, needed to simplify loop";
        Modelica.Units.SI.HeatFlowRate Q_flow[nSeg]
          "Heat exchange computed using upwind third order discretization scheme";
        //    Modelica.Units.SI.HeatFlowRate Q_flow_upWind
        //     "Heat exchange computed using upwind third order discretization scheme"; //Used to test the energy conservation
        Real sig
          "Sign used to implement the third order upwind scheme without triggering a state event";
        Real comSig
          "Sign used to implement the third order upwind scheme without triggering a state event";

      equation
        assert(nSeg >= 4,
        "Number of segments of the enhanced stratified tank should be no less than 4 (nSeg>=4).");

        // assign zero flow conditions at port
        fluidPort[:].m_flow = zeros(nSeg + 2);
        fluidPort[:].h_outflow = zeros(nSeg + 2);
        fluidPort[:].Xi_outflow = zeros(nSeg + 2, Medium.nXi);
        fluidPort[:].C_outflow = zeros(nSeg + 2, Medium.nC);

        // assign extended enthalpy vectors
        for i in 1:nSeg + 2 loop
          h[i] = inStream(fluidPort[i].h_outflow);
        end for;

        // Value that transitions between 0 and 1 as the flow reverses.
        sig = Modelica.Fluid.Utilities.regStep(
          m_flow,
          1,
          0,
          m_flow_small);
                   // at surface between port_a and vol1

        comSig = 1 - sig;

        // at surface between port_a and vol1
        hOut[1] = sig*h[1] + comSig*h[2];
        // at surface between vol[nSeg] and port_b
        hOut[nSeg + 1] = sig*h[nSeg + 1] + comSig*h[nSeg + 2];

        // Pros: These two equations can further reduce the temperature overshoot by using the upwind
        // Cons: The minimum of nSeg hase to be 4 instead of 2.
        hOut[2] = sig*h[2] + comSig*h[3];
        // at surface between vol1 and vol2
        hOut[nSeg] = sig*h[nSeg] + comSig*h[nSeg + 1];
        // at surface between vol[nSeg-1] and vol[nSeg]

        for i in 3:nSeg - 1 loop
          // at surface between vol[i-1] and vol[i]
          // QUICK method
          hOut[i] = 0.5*(h[i] + h[i + 1]) - comSig*0.125*(h[i + 2] + h[i] - 2*h[i + 1])
             - sig*0.125*(h[i - 1] + h[i + 1] - 2*h[i]);
          //     hOut[i] = 0.5*(h[i]+h[i+1]); // Central difference method
        end for;

        for i in 1:nSeg loop
          // difference between QUICK and UPWIND; index of H_flow is same as hOut
          Q_flow[i] = m_flow*(hOut[i + 1] - hOut[i]) - (H_flow[i + 1] - H_flow[i]);
        end for;

        //   Q_flow_upWind = sum(Q_flow[i] for i in 1:nSeg); //Used to test the energy conservation

        for i in 1:nSeg loop
          // Add the difference back to the volume as heat flow. An under-relaxation is needed to reduce
          // oscillations caused by high order method
          heatPort[i].Q_flow = Q_flow[i]*alpha;
        end for;
        annotation (Documentation(info="<html>
<p>
This model reduces the numerical dissipation that is introduced
by the standard first-order upwind discretization scheme which is
created when connecting fluid volumes in series.
</p>
<p>
The model is used in conjunction with
<a href=\"modelica://Buildings.Fluid.Storage.Stratified\">
Buildings.Fluid.Storage.Stratified</a>.
It computes a heat flux that needs to be added to each volume of <a href=\"modelica://Buildings.Fluid.Storage.Stratified\">
Buildings.Fluid.Storage.Stratified</a> in order to give the results that a third-order upwind discretization scheme (QUICK) would give.
</p>
<p>
The QUICK method can cause oscillations in the tank temperatures since the high order method introduces numerical dispersion.
There are two ways to reduce the oscillations:</p>
<ul>
<li>
To use an under-relaxation coefficient <code>alpha</code> when adding the heat flux into the volume.
</li>
<li>
To use the first-order upwind for <code>hOut[2]</code> and <code>hOut[nSeg]</code>. Note: Using it requires <code>nSeg &ge; 4</code>.
</li>
</ul>
<p>
Both approaches are implemented in the model.
</p>
<p>
The model is used by
<a href=\"modelica://Buildings.Fluid.Storage.StratifiedEnhanced\">
Buildings.Fluid.Storage.StratifiedEnhanced</a>.
</p>
<h4>Limitations</h4>
<p>
The model requires at least 4 fluid segments. Hence, set <code>nSeg</code> to 4 or higher.
</p>
</html>",       revisions="<html>
<ul>
<li>
June 7, 2018 by Filip Jorissen:<br/>
Copied model from Buildings and update the model accordingly.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/314\">#314</a>.
</li>
<li>
December 14, 2012 by Michael Wetter:<br/>
Removed unused protected parameters <code>sta0</code> and <code>cp0</code>.
</li>
<li>
March 29, 2012 by Wangda Zuo:<br/>
Revised the implementation to reduce the temperature overshoot.
</li>
<li>
July 28, 2010 by Wangda Zuo:<br/>
Rewrote third order upwind scheme to avoid state events.
This leads to more robust and faster simulation.
</li>
<li>
June 23, 2010 by Michael Wetter and Wangda Zuo:<br/>
First implementation.
</li>
</ul>
</html>"),       Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                  {100,100}}),graphics={
              Rectangle(
                extent={{-48,66},{48,34}},
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None),
              Rectangle(
                extent={{-48,34},{48,2}},
                fillColor={166,0,0},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None,
                lineColor={0,0,0}),
              Rectangle(
                extent={{-48,2},{48,-64}},
                fillColor={0,0,255},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None,
                lineColor={0,0,0})}));
      end ThirdOrderStratifier;

      package Examples "Examples for BaseClasses models"
        extends Modelica.Icons.ExamplesPackage;

        model IndirectTankHeatExchanger
          "Example showing the use of IndirectTankHeatExchanger"
          extends Modelica.Icons.Example;

          package Medium = Buildings.Media.Water "Buildings library model for water";

          ProsNet.Under_Development.Storage.Storage.BaseClasses.IndirectTankHeatExchanger
            indTanHex(
            nSeg=3,
            CHex=50,
            Q_flow_nominal=3000,
            m_flow_nominal=3000/20/4200,
            volHexFlu=0.0004,
            dExtHex=0.01905,
            redeclare package MediumTan = Medium,
            redeclare package MediumHex = Medium,
            dp_nominal=10000,
            energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
            TTan_nominal=293.15,
            THex_nominal=323.15) "Heat exchanger" annotation (Placement(
                transformation(
                extent={{-12,-17},{12,17}},
                rotation=90,
                origin={-19,8})));

          ProsNet.Fluid.Building_Fluid.Sources.Boundary_pT bou1(nPorts=1,
              redeclare package Medium = Medium) annotation (Placement(
                transformation(extent={{-72,-42},{-52,-22}})));
          ProsNet.Fluid.Building_Fluid.Sources.MassFlowSource_T bou(
            m_flow=0.1,
            nPorts=1,
            redeclare package Medium = Medium,
            T=323.15)
            annotation (Placement(transformation(extent={{-72,34},{-52,54}})));

          Buildings.HeatTransfer.Sources.FixedTemperature watTem[3](each T=293.15)
            annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=180,
                origin={30,8})));
        equation
          connect(bou1.ports[1], indTanHex.port_a)
            annotation (Line(
              points={{-52,-32},{-19,-32},{-19,-4}},
              color={0,127,255},
              smooth=Smooth.None));
          connect(bou.ports[1], indTanHex.port_b)
            annotation (Line(
              points={{-52,44},{-19,44},{-19,20}},
              color={0,127,255},
              smooth=Smooth.None));

          connect(watTem.port, indTanHex.port)
            annotation (Line(
              points={{20,8},{-7.89333,8}},
              color={191,0,0},
              smooth=Smooth.None));
          annotation (__Dymola_Commands(file=
                  "modelica://Buildings/Resources/Scripts/Dymola/Fluid/Storage/BaseClasses/Examples/IndirectTankHeatExchanger.mos"
                "Simulate and plot"),
                experiment(Tolerance=1e-6, StopTime=15),
        Documentation(info="<html>
<p>
This model provides an example of how the
<a href=\"modelica://Buildings.Fluid.Storage.BaseClasses.IndirectTankHeatExchanger\">
Buildings.Fluid.Storage.BaseClasses.IndirectTankHeatExchanger</a>
model is used. In the model water flows from a flow source through
the heat exchanger to a low pressure environment. The stagnant fluid on the outside
of the heat exchanger is modeled as a constant temperature.<br/>
</p>
</html>",
        revisions="<html>
<ul>
<li>
June 7, 2018 by Filip Jorissen:<br/>
Copied model from Buildings and update the model accordingly.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/314\">#314</a>.
</li>
<li>
February 27, 2016 by Michael Wetter:<br/>
Stored example in a single file rather than a file with multiple examples.
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/488\">#488</a>.
</li>
<li>
December 22, 2014 by Michael Wetter:<br/>
Removed <code>Modelica.Fluid.System</code>
to address issue
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/311\">#311</a>.
</li>
<li>
March 27, 2013 by Peter Grant:<br/>
First implementation
</li>
</ul>
</html>"));
        end IndirectTankHeatExchanger;
      annotation(Documentation(info="<html>
<p>
This package contains examples for models found in <a href=\"modelica://Buildings.Fluid.Storage.BaseClasses\">
Buildings.Fluid.Storage.BaseClasses</a>.
</p>
</html>"));
      end Examples;
    annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains base classes that are used to construct the models in
<a href=\"modelica://Buildings.Fluid.Storage\">Buildings.Fluid.Storage</a>.
</p>
</html>"));
    end BaseClasses;
  annotation (preferredView="info", Documentation(info="<html>
This package contains thermal energy storage models.
</html>"));
  end Storage;
end Storage;
