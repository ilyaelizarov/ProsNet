﻿within ProsNet.Prosumers.BaseClasses;
model PrimarySide_new

  extends ProsNet.Fluid.Interfaces.PartialFourPortInterface(
   final m1_flow_nominal=m_flow_nominal_1,
   final m2_flow_nominal=m_flow_nominal_2);

  // Heat exchanger - thermal parameters
  parameter Modelica.SIunits.HeatFlowRate Q_flow_nominal=30000 "Heat flow rate"
   annotation(Dialog(group="Nominal parameters of heat exchanger"));
  parameter Modelica.SIunits.Temperature T_a1_nominal=343.15 "Inlet temperature on the primary side"
   annotation(Dialog(group="Heat exchanger"));
  parameter Modelica.SIunits.Temperature T_a2_nominal=318.15 "Inlet temperature on the secondary side"
   annotation(Dialog(group="Heat exchanger"));
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal_1=0.358 "Primary side mass flow rate"
    annotation(Dialog(group="Heat exchanger"));
  parameter Modelica.SIunits.PressureDifference dp1_nominal=155e2 "Primary side pressure loss"
   annotation(Dialog(group="Heat exchanger"));
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal_2=0.358 "Secondary side mass flow rate"
   annotation(Dialog(group="Heat exchanger"));
  parameter Modelica.SIunits.PressureDifference dp2_nominal=155e2 "Secondary side pressure loss"
 annotation(Dialog(group="Heat exchanger"));
  parameter Real r_nominal(min=0, max=1) = (m1_flow_nominal/m2_flow_nominal)^n
   "Ratio between primary side and secondary side convective heat transfer coefficient" annotation (
    Dialog(tab = "Advanced", group = "Heat transfer in heat exchanger"));
  parameter Real n(min=0, max=1) = 0.6 "Exponent coefficient" annotation (
    Dialog(tab = "Advanced", group = "Heat transfer in heat exchanger"));

  // Feed-in pump parameters
  replaceable parameter ProsNet.Fluid.Pumps.Data.Generic feedinPer
   constrainedby ProsNet.Fluid.Pumps.Data.Generic "Record with performance data"
    annotation (Dialog(group="Feed-in pump"),choicesAllMatching=true);
  // Feed-in pump dynamic options
  parameter Modelica.Fluid.Types.Dynamics energyDynamics_feedPump = Modelica.Fluid.Types.Dynamics.SteadyState
   "Energy dynamics" annotation(Dialog(tab="Dynamics", group="Feed-in pump"));
  parameter Modelica.SIunits.Time tau_feedPump = 1 "Time constant for energy content inside the pump"
   annotation(Dialog(tab="Dynamics", group="Feed-in pump"));
  parameter Boolean use_inputFilter_feedPump = false "Activate start-up and shut-down transients"
   annotation(Dialog(tab="Dynamics", group="Feed-in pump"));
  parameter Modelica.SIunits.Time riseTime_feedPump(min=0) = 5 "Rise time for the transients"
   annotation(Dialog(tab="Dynamics", group="Feed-in pump"));
  parameter Modelica.Blocks.Types.Init init_feedPump=Modelica.Blocks.Types.Init.InitialOutput "Type of initialization"
   annotation(Dialog(tab="Dynamics", group="Feed-in pump"));
  parameter Modelica.SIunits.MassFlowRate y_start_feedPump=0 "Initial value of mass flow rate"
   annotation(Dialog(tab="Dynamics", group="Feed-in pump"));

  // Control valve parameters
  parameter Real Kv_conVal=2.5 "Kv (metric) flow coefficient [m3h/bar^(1/2)]"
   annotation(Dialog(group="Control valve"));
  parameter Real l_conVal=2e-3 "Valve leakage, l=Kv(y=0)/Kv(y=1)"
   annotation(Dialog(group="Control valve"));
  // Dynamics of the valve
  parameter Boolean use_inputFilter_conVal = false "Transient behavior for moving the valve’s stem"
   annotation(Dialog(tab="Dynamics", group="Control valve"));
  parameter Modelica.SIunits.Time riseTime_conVal(min=0) = 5 "Rise time"
   annotation(Dialog(tab="Dynamics", group="Control valve"));
  parameter Modelica.Blocks.Types.Init init_conVal=Modelica.Blocks.Types.Init.InitialOutput "Type of initialization"
   annotation(Dialog(tab="Dynamics", group="Control valve"));
  parameter Modelica.SIunits.MassFlowRate m_flow_start_conVal=0 "Initial value of mass flow rate"
   annotation(Dialog(tab="Dynamics", group="Control valve"));
  parameter Real y_start_conVal=1 "Initial value of output"
    annotation(Dialog(tab="Dynamics", group="Control valve"));

  // Check valve parameres
  parameter Real Kv_cheVal=6 "Kv (metric) flow coefficient [m3h/bar^(1/2)]"
   annotation(Dialog(group="Check valve"));
  parameter Real l_cheVal=0.001 "Valve leakage, l=Kv(y=0)/Kv(y=1)"
   annotation(Dialog(group="Check valve"));

  ProsNet.Fluid.HeatExchangers.LiquidToLiquid HEX(final m1_flow_nominal=m_flow_nominal_1,
   final m2_flow_nominal=m_flow_nominal_2,
   redeclare final package Medium1=Medium1,
   redeclare final package Medium2=Medium2,
   final dp1_nominal=dp1_nominal,
   final dp2_nominal=dp2_nominal,
   final Q_flow_nominal=Q_flow_nominal,
   final T_a1_nominal=T_a1_nominal,
   final T_a2_nominal=T_a2_nominal)                    annotation (Placement(transformation(
        extent={{-14,-17},{14,17}},
        rotation=0,
        origin={48,3})));

  ProsNet.Fluid.Valves.TwoWayEqualPercentage val(m_flow_nominal=m_flow_nominal_1,
   redeclare final package Medium=Medium1,
   final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
   final Kv=Kv_conVal,
   final use_inputFilter=use_inputFilter_conVal,
   final riseTime=riseTime_conVal,
   final init=init_conVal,
   final y_start=y_start_conVal,
   final l=l_conVal) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={-40,10})));

  ProsNet.Fluid.FixedResistances.CheckValve cheVal(m_flow_nominal=m_flow_nominal_1,
   redeclare final package Medium=Medium1,
   final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
   final Kv=Kv_cheVal,
   final l=l_cheVal) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-66,26})));

  ProsNet.Fluid.Pumps.SpeedControlled_y feedPump(redeclare final package Medium=Medium1,
    final energyDynamics=energyDynamics_feedPump,
    final tau=tau_feedPump,
    final per = feedinPer,
    final use_inputFilter=use_inputFilter_feedPump,
    final riseTime=riseTime_feedPump,
    final init=init_feedPump,
    final y_start=y_start_feedPump)
     annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-66,-12})));

  Modelica.Blocks.Interfaces.IntegerInput mu "Operation mode"
    annotation (Placement(transformation(extent={{-140,138},{-100,178}})));
  Modelica.Blocks.Interfaces.IntegerInput pi "Participation"
    annotation (Placement(transformation(extent={{-140,100},{-100,140}})));
  Modelica.Blocks.Interfaces.RealInput conVal_op_set
    "Normalized control valve opening" annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={-40,120})));
  Modelica.Blocks.Interfaces.RealInput feedPump_y_set
    "Normalized rotational speed of a feed-in pump"   annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={40,120})));
  Controls.PrimaryFlowControl priFlowCon
    annotation (Placement(transformation(extent={{8,36},{-12,56}})));

equation

  connect(port_a1, val.port_b)
    annotation (Line(points={{-100,60},{-40,60},{-40,20}}, color={0,127,255}));
  connect(cheVal.port_a, port_a1)
    annotation (Line(points={{-66,36},{-66,60},{-100,60}}, color={0,127,255}));
  connect(cheVal.port_b,feedPump. port_a)
    annotation (Line(points={{-66,16},{-66,-2}}, color={0,127,255}));
  connect(feedPump.port_b, val.port_a) annotation (Line(points={{-66,-22},{-66,-32},
          {-40,-32},{-40,0}}, color={0,127,255}));
  connect(HEX.port_a1, val.port_a) annotation (Line(points={{34,13.2},{16,13.2},
          {16,14},{-2,14},{-2,-32},{-40,-32},{-40,1.77636e-15}}, color={0,127,255}));
  connect(port_b1, HEX.port_b1) annotation (Line(points={{100,60},{82,60},{82,14},
          {62,14},{62,13.2}}, color={0,127,255}));
  connect(HEX.port_a2, port_a2) annotation (Line(points={{62,-7.2},{72,-7.2},{72,
          -60},{100,-60}}, color={0,127,255}));
  connect(HEX.port_b2, port_b2) annotation (Line(points={{34,-7.2},{26,-7.2},{26,
          -8},{16,-8},{16,-60},{-100,-60}}, color={0,127,255}));
  connect(priFlowCon.valve_op, val.y) annotation (Line(points={{-13,51.1},{-22,51.1},{-22,10},{
          -28,10}}, color={0,0,127}));
  connect(priFlowCon.pump_y, feedPump.y) annotation (Line(points={{-13,40.9},{-16,40.9},{-16,-28},
          {-88,-28},{-88,-12},{-78,-12}}, color={0,0,127}));
  connect(conVal_op_set, priFlowCon.valve_op_set)
    annotation (Line(points={{-40,120},{-40,92},{-6,92},{-6,58}}, color={0,0,127}));
  connect(feedPump_y_set, priFlowCon.pump_y_set)
    annotation (Line(points={{40,120},{40,92},{2,92},{2,58}}, color={0,0,127}));
  connect(mu, priFlowCon.mu) annotation (Line(points={{-120,158},{-62,158},{-62,74},{24,74},{24,
          52},{10,52}}, color={255,127,0}));
  connect(pi, priFlowCon.pi) annotation (Line(points={{-120,120},{-74,120},{-74,66},{20,66},{20,
          40},{10,40}}, color={255,127,0}));
  annotation (defaultComponentName="priSide", Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-60,66},{56,30}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-60,30},{56,66}}, color={28,108,200}),
        Ellipse(
          extent={{-58,-22},{-18,-62}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{18,-16},{50,-16},{34,-42},{18,-16}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{18,-68},{50,-68},{34,-42},{18,-68}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-46,-34},{-30,-34},{-38,-22},{-46,-34}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Line(points={{-38,-22},{-38,-10}},
                                         color={28,108,200}),
        Line(points={{-38,-62},{-38,-74}}, color={28,108,200}),
        Line(points={{34,-16},{34,-4}},
                                      color={28,108,200}),
        Line(points={{34,-68},{34,-82}}, color={28,108,200}),
        Line(points={{-44,66},{-44,80}}, color={28,108,200}),
        Line(points={{30,66},{30,80}}, color={28,108,200}),
        Line(points={{-44,30},{-44,18}}, color={28,108,200}),
        Line(points={{30,30},{30,18}}, color={28,108,200}),
        Line(points={{14,-42},{34,-42},{28,-38}}, color={28,108,200}),
        Line(points={{14,-42},{34,-42},{28,-46}}, color={28,108,200})}),
                                                                 Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end PrimarySide_new;
