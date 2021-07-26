within ProsNet.Fluid.HeatExchangers.BaseClasses;
model HAPlateHE
  "Calculates hA_1 and hA_2 values for a heat exchanger"
  extends Modelica.Blocks.Icons.Block;

  parameter Modelica.Units.SI.MassFlowRate m_flow_nominal_primary
    "Water mass flow rate on primary side"
    annotation (Dialog(tab="General", group="Nominal condition"));
  parameter Modelica.Units.SI.MassFlowRate m_flow_nominal_secondary
    "Water mass flow rate on secondary side"
    annotation (Dialog(tab="General", group="Nominal condition"));

  parameter Modelica.Units.SI.ThermalConductance UA_nominal(min=0)
    "Nominal thermal conductance"
    annotation (Dialog(tab="General", group="Nominal condition"));

/*     parameter Real r_nominal(min = 0) = 1 "Ratio between primary-side and secondary-side convective heat transfer coefficient: r=(m_nom1/m_nom2)^n" annotation (
Dialog(tab = "General", group = "Nominal condition"));*/

     parameter Real r_nominal(min = 0) "Ratio between primary-side and secondary-side convective heat transfer coefficient: r=(m_nom1/m_nom2)^n" annotation (
    Dialog(tab = "General", group = "Nominal condition"));

     parameter Real n  "Exponent coefficient" annotation (
    Dialog(tab = "Advanced", group = "Parameters"));

     parameter Real widthSmoothing =  0.05 "Width of transition interval between alpha_min and alpha" annotation (
    Dialog(tab = "Advanced", group = "Parameters"));

  parameter Modelica.Units.SI.MassFlowRate m_flow_small=abs(
      m_flow_nominal_primary)*1e-4
    "Small mass flow rate for regularization of zero flow"
    annotation (Dialog(tab="Advanced"));

  Modelica.Blocks.Interfaces.RealOutput alphaA_1(unit = "W/K") "hA value" annotation (
    Placement(visible = true, transformation(extent = {{100, 66}, {120, 86}}, rotation = 0), iconTransformation(extent = {{100, 62}, {120, 82}}, rotation = 0)));

  Modelica.Blocks.Interfaces.RealOutput alphaA_2(unit = "W/K") "hA value" annotation (
    Placement(visible = true, transformation(extent = {{100, -84}, {120, -64}}, rotation = 0), iconTransformation(extent = {{100, -84}, {120, -64}}, rotation = 0)));

  Modelica.Blocks.Interfaces.RealInput m_flow_primary(start = 0.01, unit = "kg/s") "Mass flow rate on primary side" annotation (
    Placement(visible = true, transformation(extent = {{-120, 70}, {-100, 90}}, rotation = 0), iconTransformation(extent = {{-120, 66}, {-100, 86}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput m_flow_secondary(start = 0.01, unit = "kg/s") "Mass flow rate on secondary side" annotation (
    Placement(visible = true, transformation(extent = {{-120, -72}, {-100, -52}}, rotation = 0), iconTransformation(extent = {{-120, -80}, {-100, -60}}, rotation = 0)));

  // Variables
  Modelica.Units.SI.MassFlowRate m_flow1=Modelica.Fluid.Utilities.regStep(
      m_flow_primary,
      m_flow_primary,
      -m_flow_primary,
      m_flow_small);

  Modelica.Units.SI.MassFlowRate m_flow2=Modelica.Fluid.Utilities.regStep(
      m_flow_secondary,
      m_flow_secondary,
      -m_flow_secondary,
      m_flow_small);

  Modelica.Units.SI.ThermalConductance alphaA_1_nominal;

  Modelica.Units.SI.ThermalConductance alphaA_2_nominal;

  Modelica.Blocks.Sources.RealExpression alpha1A(y = alphaA_1_nominal * (m_flow1 / m_flow_nominal_primary) ^ n)  annotation (
    Placement(visible = true, transformation(origin = {-50, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression alpha2A(y = alphaA_2_nominal * (m_flow2 / m_flow_nominal_secondary) ^ n) annotation (
    Placement(visible = true, transformation(origin = {-50, -44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ProsNet.Utilities.Math.SmoothMax smoothMax1(deltaX = widthSmoothing)  annotation (
    Placement(visible = true, transformation(origin = {10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ProsNet.Utilities.Math.SmoothMax smoothMax2(deltaX = widthSmoothing)  annotation (
    Placement(visible = true, transformation(origin = {10, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant freeConvection(k = 50)  annotation (
    Placement(visible = true, transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(alpha1A.y, smoothMax1.u1) annotation (
    Line(points = {{-39, 44}, {-20.5, 44}, {-20.5, 36}, {-2, 36}}, color = {0, 0, 127}));
  connect(alpha2A.y, smoothMax2.u1) annotation (
    Line(points = {{-39, -44}, {-20.5, -44}, {-20.5, -24}, {-2, -24}}, color = {0, 0, 127}));
  connect(smoothMax2.y, alphaA_2) annotation (
    Line(points={{21,-30},{60,-30},{60,-74},{110,-74},{110,-74}},            color = {0, 0, 127}));
  connect(smoothMax1.y, alphaA_1) annotation (
    Line(points={{21,30},{60,30},{60,76},{110,76},{110,76}},            color = {0, 0, 127}));
  connect(freeConvection.y, smoothMax2.u2) annotation (
    Line(points={{-19,0},{-12,0},{-12,-36},{-2,-36},{-2,-36}},            color = {0, 0, 127}));
  connect(freeConvection.y, smoothMax1.u2) annotation (
    Line(points={{-19,0},{-12,0},{-12,24},{-2,24},{-2,24}},            color = {0, 0, 127}));

  alphaA_1_nominal = (r_nominal + 1) * UA_nominal;
  alphaA_2_nominal = UA_nominal * (r_nominal + 1) / r_nominal;

  annotation (
    Dialog(tab = "Advanced", group = "Parameters"),
    Icon(graphics={  Rectangle(origin={-1,-47},    rotation = -90,
            lineThickness =                                                        1.5, extent = {{-37, 63}, {37, -63}}), Line(origin = {-0.829457, -49.7907}, rotation = -90, points = {{-37, 63}, {37, -63}}, thickness = 0.75), Text(extent = {{-72, 98}, {60, -16}}, lineColor = {0, 0, 0}, textString = "hA")}, coordinateSystem(initialScale = 0.1)));
end HAPlateHE;
