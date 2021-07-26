within ProsNet.Prosumers.SecondarySides;
model PrescribedSecondarySide

  extends ProsNet.Prosumers.SecondarySides.BaseClasses.PumpsPairPartial(
    final m_flow_nominal = m_flow_nominal_cv);

  // Nominal conditions
  parameter Modelica.Units.SI.MassFlowRate m_flow_nominal_cv
    "Nominal flow rate" annotation (Dialog(group=
          "Secondary side control volume nominal conditions"));
  parameter Modelica.Units.SI.PressureDifference dp_nominal_cv=0
    "Nominal pressure difference" annotation (Dialog(group=
          "Secondary side control volume nominal conditions"));

  // Dynamic parameters for CV in PrescribedSecondarySide
  extends ProsNet.Prosumers.SecondarySides.BaseClasses.ControlVolumeDynParam;

  Modelica.Blocks.Interfaces.RealInput T_set(unit="K", displayUnit="degC")
    "Temperature set point"   annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={-40,120})));
  Modelica.Blocks.Interfaces.RealInput m_flow_set(unit="kg/s", displayUnit="kg/s")
    "Mass flow rate set point"  annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={40,120})));
  ProsNet.Controls.SecondaryFlowControl secFlCon
    annotation (Placement(transformation(extent={{-60,-42},{-40,-22}})));
  ProsNet.Fluid.HeatExchangers.ControlVolume_T CV_T(redeclare final package Medium = Medium,
    m_flow_nominal=m_flow_nominal_cv,
    dp_nominal=dp_nominal_cv,
    tau=tau_cv,
    T_start=T_start_cv,
    energyDynamics=energyDynamics_cv)
    annotation (Placement(transformation(extent={{-10,42},{10,62}})));

equation
  connect(CV_T.port_b, pump_prod.port_a) annotation (Line(points={{10,52},{26,52},{26,-22},{52,
          -22},{52,-32}}, color={0,127,255}));
  connect(port_a, CV_T.port_a)
    annotation (Line(points={{-100,0},{-44,0},{-44,52},{-10,52}}, color={0,127,255}));
  connect(pi, secFlCon.pi) annotation (Line(points={{-120,112},{-86,112},{-86,-38},{-62,-38}},
        color={255,127,0}));
  connect(mu, secFlCon.mu) annotation (Line(points={{-120,150},{-76,150},{-76,-26},{-62,-26}},
        color={255,127,0}));
  connect(T_set, CV_T.TSet)
    annotation (Line(points={{-40,120},{-40,60},{-12,60}}, color={0,0,127}));
  connect(m_flow_set, secFlCon.m_flow_set)
    annotation (Line(points={{40,120},{40,-8},{-50,-8},{-50,-20}}, color={0,0,127}));
  connect(secFlCon.m_flow_consumption, pump_cons.m_flow_in) annotation (Line(points={{-39,-37.1},
          {-26,-37.1},{-26,-42},{-10,-42}}, color={0,0,127}));
  connect(secFlCon.m_flow_production, pump_prod.m_flow_in) annotation (Line(points={{-39,-26.9},
          {-14,-26.9},{-14,-16},{74,-16},{74,-42},{64,-42}}, color={0,0,127}));
  annotation (Icon(graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-58,20},{-18,-20}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-46,8},{-30,8},{-38,20},{-46,8}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Line(points={{-38,20},{-38,32}}, color={28,108,200}),
        Line(points={{-38,-20},{-38,-32}}, color={28,108,200}),
        Polygon(
          points={{10,-26},{72,-26},{60,28},{56,2},{46,12},{42,20},{36,4},{32,22},
              {28,-2},{18,28},{10,-26}},
          lineColor={238,46,47},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{52,-42},{52,-88},{72,-62},{52,-42}},
          lineColor={28,108,200},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-72,-78},{52,-78},{52,-52},{-72,-78}},
          lineColor={28,108,200},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{50,-52},{-72,-52},{-72,-78},{50,-52}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid)}));
end PrescribedSecondarySide;
