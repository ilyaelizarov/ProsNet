within ProsNet.Prosumers.BaseClasses;
partial class PrimarySideParameters

  // Heat exchanger - thermal parameters
  parameter Modelica.Units.SI.HeatFlowRate Q_flow_nominal=30000
    "Heat flow rate"
    annotation (Dialog(group="Nominal conditions for heat exchanger"));
  parameter Modelica.Units.SI.Temperature T_a1_nominal=343.15
    "Inlet temperature on the primary side"
    annotation (Dialog(group="Nominal conditions for heat exchanger"));
  parameter Modelica.Units.SI.Temperature T_a2_nominal=318.15
    "Inlet temperature on the secondary side"
    annotation (Dialog(group="Nominal conditions for heat exchanger"));
  parameter Modelica.Units.SI.MassFlowRate m_flow_nominal_1=0.358
    "Primary side mass flow rate"
    annotation (Dialog(group="Nominal conditions for heat exchanger"));
  parameter Modelica.Units.SI.PressureDifference dp1_nominal=155e2
    "Primary side pressure loss"
    annotation (Dialog(group="Nominal conditions for heat exchanger"));
  parameter Modelica.Units.SI.MassFlowRate m_flow_nominal_2=0.358
    "Secondary side mass flow rate"
    annotation (Dialog(group="Nominal conditions for heat exchanger"));
  parameter Modelica.Units.SI.PressureDifference dp2_nominal=155e2
    "Secondary side pressure loss"
    annotation (Dialog(group="Nominal conditions for heat exchanger"));
  parameter Real r_nominal(min=0, max=1) = (m_flow_nominal_1/m_flow_nominal_2)^n
   "Ratio between primary side and secondary side convective heat transfer coefficient" annotation (
    Dialog(tab = "Advanced", group = "Heat transfer in heat exchanger"));
  parameter Real n(min=0, max=1) = 0.6 "Exponent coefficient" annotation (
    Dialog(tab = "Advanced", group = "Heat transfer in heat exchanger"));

  // Feed-in pump parameters
  replaceable parameter ProsNet.Fluid.Pumps.Data.Pumps.IMP.NMTPlus152025to40 feedinPer
   constrainedby ProsNet.Fluid.Pumps.Data.Generic "Record with performance data"
    annotation (Dialog(group="Feed-in pump"),choicesAllMatching=true);
  // Feed-in pump dynamic options
  parameter Modelica.Fluid.Types.Dynamics energyDynamics_feedPump = Modelica.Fluid.Types.Dynamics.SteadyState
   "Energy dynamics" annotation(Dialog(tab="Dynamics", group="Feed-in pump"));
  parameter Modelica.Units.SI.Time tau_feedPump=1
    "Time constant for energy content inside the pump"
    annotation (Dialog(tab="Dynamics", group="Feed-in pump"));
  parameter Boolean use_inputFilter_feedPump = false "Activate start-up and shut-down transients"
   annotation(Dialog(tab="Dynamics", group="Feed-in pump"));
  parameter Modelica.Units.SI.Time riseTime_feedPump(min=0) = 5
    "Rise time for the transients"
    annotation (Dialog(tab="Dynamics", group="Feed-in pump"));
  parameter Modelica.Blocks.Types.Init init_feedPump=Modelica.Blocks.Types.Init.InitialOutput "Type of initialization"
   annotation(Dialog(tab="Dynamics", group="Feed-in pump"));
  parameter Real y_start_feedPump=0 "Initial value of speed"
   annotation(Dialog(tab="Dynamics", group="Feed-in pump"));

  // Control valve parameters
  parameter Real Kv_conVal=2.5 "Kv (metric) flow coefficient [m3h/bar^(1/2)]"
   annotation(Dialog(group="Control valve"));
  parameter Real l_conVal=2e-3 "Valve leakage, l=Kv(y=0)/Kv(y=1)"
   annotation(Dialog(group="Control valve"));
  // Dynamics of the valve
  parameter Boolean use_inputFilter_conVal = false "Transient behavior for moving the valve’s stem"
   annotation(Dialog(tab="Dynamics", group="Control valve"));
  parameter Modelica.Units.SI.Time riseTime_conVal(min=0) = 5 "Rise time"
    annotation (Dialog(tab="Dynamics", group="Control valve"));
  parameter Modelica.Blocks.Types.Init init_conVal=Modelica.Blocks.Types.Init.InitialOutput "Type of initialization"
   annotation(Dialog(tab="Dynamics", group="Control valve"));
  parameter Real y_start_conVal=1 "Initial value of output"
    annotation(Dialog(tab="Dynamics", group="Control valve"));

  // Check valve parameres
  parameter Real Kv_cheVal=6 "Kv (metric) flow coefficient [m3h/bar^(1/2)]"
   annotation(Dialog(group="Check valve"));
  parameter Real l_cheVal=0.001 "Valve leakage, l=Kv(y=0)/Kv(y=1)"
   annotation(Dialog(group="Check valve"));

  // Additional hydraulic losses
  parameter Modelica.Units.SI.PressureDifference preDro_pros=4.27e3
    "Additional hydraulic losses on the primary side"
    annotation (Dialog(group="Local hydraulic losses"));

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end PrimarySideParameters;
