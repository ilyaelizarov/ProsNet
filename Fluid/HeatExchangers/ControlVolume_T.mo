within ProsNet.Fluid.HeatExchangers;
model ControlVolume_T "Heater or cooler with prescribed outlet temperature"
  extends ProsNet.Fluid.Interfaces.TwoPortHeatMassExchanger(
    redeclare final ProsNet.Fluid.MixingVolumes.MixingVolume vol(
    final prescribedHeatFlowRate=false),
    dp_nominal = 0);

  Modelica.Blocks.Interfaces.RealInput TSet(unit="K", displayUnit="degC")
    "Set point temperature of the fluid that leaves port_b"
    annotation (Placement(transformation(origin={-120,80},
              extent={{20,-20},{-20,20}},rotation=180)));
  Modelica.Blocks.Interfaces.RealOutput Q_flow_trnsf(unit="W")
    "Heat flow rate added to the fluid (if flow is from port_a to port_b)"
    annotation (Placement(transformation(extent={{100,70},{120,90}}),
        iconTransformation(extent={{100,70},{120,90}})));
  Modelica.Blocks.Sources.RealExpression exprQ_flow_(y=port_a.m_flow*(port_b.h_outflow
         - inStream(port_a.h_outflow)))
    annotation (Placement(transformation(extent={{30,38},{52,58}})));
  Modelica.Blocks.Sources.RealExpression HeatFlowRate(y=Q_flow)
    annotation (Placement(transformation(extent={{-86,38},{-64,58}})));
protected
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow preHea(final alpha=0)
    "Prescribed heat flow"
    annotation (Placement(transformation(extent={{-50,38},{-30,58}})));

  Modelica.Units.SI.SpecificEnthalpy hSet "Set point for enthalpy";

  Modelica.Units.SI.HeatFlowRate Q_flow;

equation

  // Specific heat flow rate that corresponds to the input temperature
  hSet = Medium.specificEnthalpy(Medium.setState_pTX(p = port_a.p,
      T = TSet));

  // Heat flow rate to get to the input temperature
  Q_flow = Modelica.Fluid.Utilities.regStep(port_a.m_flow,
   port_a.m_flow*(hSet - inStream(port_a.h_outflow)),
    port_b.m_flow*(hSet - inStream(port_b.h_outflow)), m_flow_small);


  connect(preHea.port, vol.heatPort) annotation (Line(points={{-30,48},{-20,48},
          {-20,-10},{-9.4,-10}}, color={191,0,0}));
  connect(HeatFlowRate.y, preHea.Q_flow)
    annotation (Line(points={{-62.9,48},{-50,48}}, color={0,0,127}));
  connect(exprQ_flow_.y, Q_flow_trnsf) annotation (Line(points={{53.1,48},{86,
          48},{86,80},{110,80}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,82},{-70,78}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-70,60},{-66,82}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-110,102},{-74,84}},
          lineColor={0,0,127},
          textString="T"),
        Text(
          extent={{26,108},{94,84}},
          lineColor={0,0,127},
          textString="Q_flow"),
        Rectangle(
          extent={{70,82},{100,78}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{66,60},{70,82}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-70,60},{-70,-60},{70,60},{-70,60}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-70,-60},{70,60},{70,-60},{-70,-60}},
          lineColor={28,108,200},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid)}),                      Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ControlVolume_T;
