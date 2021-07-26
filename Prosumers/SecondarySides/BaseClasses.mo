within ProsNet.Prosumers.SecondarySides;
package BaseClasses
    extends Modelica.Icons.BasesPackage;

  partial model PumpsPairPartial

    extends ProsNet.Fluid.Interfaces.PartialTwoPort;

    // PumpsPairPartial dynamic parameters
    extends ProsNet.Prosumers.SecondarySides.BaseClasses.PumpsPairDynParam;

    // Parameters
    parameter Modelica.Units.SI.MassFlowRate m_flow_nominal "Nominal flow rate"
      annotation (Dialog(group="Secondary side pumps nominal conditions"));
    parameter Modelica.Units.SI.PressureDifference dp_nominal
      "Nominal pressure difference"
      annotation (Dialog(group="Secondary side pumps nominal conditions"));

    ProsNet.Fluid.Pumps.FlowControlled_m_flow pump_prod(final energyDynamics=energyDynamics_pumpsSec,
        final tau=tau_pumpsSec,
        final m_flow_nominal=m_flow_nominal,
        final use_inputFilter=use_inputFilter_pumpsSec,
        final riseTime=riseTime_pumpsSec,
        final dp_nominal=dp_nominal,
        final m_flow_start=m_flow_start_pumpsSec,
        final y_start=y_start_pumpsSec) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={52,-42})));

    ProsNet.Fluid.Pumps.FlowControlled_m_flow pump_cons(final energyDynamics=energyDynamics_pumpsSec,
        final tau=tau_pumpsSec,
        final m_flow_nominal=m_flow_nominal,
        final use_inputFilter=use_inputFilter_pumpsSec,
        final riseTime=riseTime_pumpsSec,
        final dp_nominal=dp_nominal,
        final m_flow_start=m_flow_start_pumpsSec,
        final y_start=y_start_pumpsSec) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={2,-42})));

    Modelica.Blocks.Interfaces.IntegerInput mu "Operating mode"
                                               annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=0,
          origin={-120,150})));

    Modelica.Blocks.Interfaces.IntegerInput pi "Participation"
                                               annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=0,
          origin={-120,112})));

  equation
    connect(pump_cons.port_a, pump_prod.port_b) annotation (Line(points={{2,-52},{
            2,-62},{52,-62},{52,-52}},
                                   color={0,127,255}));
    connect(pump_cons.port_b, pump_prod.port_a) annotation (Line(points={{2,-32},{
            2,-22},{52,-22},{52,-32}},
                                     color={0,127,255}));
    connect(port_b, pump_prod.port_b) annotation (Line(points={{100,0},{80,0},{80,-82},{28,-82},{
            28,-62},{52,-62},{52,-52}}, color={0,127,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
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
          Line(points={{-38,-20},{-38,-32}}, color={28,108,200})}),Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end PumpsPairPartial;

  partial class PumpsPairDynParam

    parameter Modelica.Fluid.Types.Dynamics energyDynamics_pumpsSec = Modelica.Fluid.Types.Dynamics.SteadyState
      "Energy dynamics" annotation(Dialog(tab="Dynamics", group="Secondary side pumps dynamics"));
    parameter Modelica.Units.SI.Time tau_pumpsSec(min=0) = 10
      "Time constant of fluid volume in pumps" annotation (Dialog(tab=
            "Dynamics", group="Secondary side pumps dynamics"));
    parameter Boolean use_inputFilter_pumpsSec = false "Start-up and shut-down dynamics of pumps"
     annotation(Dialog(tab="Dynamics", group="Secondary side pumps dynamics"));
    parameter Modelica.Units.SI.Time riseTime_pumpsSec(min=0) = 5 "Rise time"
      annotation (Dialog(tab="Dynamics", group="Secondary side pumps dynamics"));
    parameter Modelica.Units.SI.MassFlowRate m_flow_start_pumpsSec=0
      "Initial value of mass flow rate" annotation (Dialog(tab="Dynamics",
          group="Secondary side pumps dynamics"));
    parameter Real y_start_pumpsSec = 1 "Initial value of speed"
      annotation(Dialog(tab="Dynamics", group="Secondary side pumps dynamics"));

  end PumpsPairDynParam;

  partial class ControlVolumeDynParam

    parameter Modelica.Fluid.Types.Dynamics energyDynamics_cv = Modelica.Fluid.Types.Dynamics.SteadyState
      "Energy dynamics" annotation(Dialog(tab="Dynamics", group="Secondary side control volume dynamics"));
    parameter Modelica.Units.SI.Time tau_cv(min=0) = 10
      "Time constant for control volume" annotation (Dialog(tab="Dynamics",
          group="Secondary side control volume dynamics"));
    parameter Modelica.Units.SI.Temperature T_start_cv=293.15
      "Initial temperature inside volume" annotation (Dialog(tab="Dynamics",
          group="Secondary side control volume dynamics"));

  end ControlVolumeDynParam;
end BaseClasses;
