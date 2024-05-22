within ProsNet.Prosumers;
model ProsumerIdeal

  extends ProsNet.Prosumers.BaseClasses.PrimarySidePartial;

  Modelica.Units.SI.MassFlowRate m1_flow_small;

  /* Parameters */
  // Activates conditional inputs
  parameter Boolean use_T_set_in = false "Get temperature set point for the secondary side from a connector"
    annotation(Dialog(group="Conditional control inputs"));
  parameter Boolean use_m_flow_set_in = false "Get mass flow rate for the secondary side from a connector"
    annotation(Dialog(group="Conditional control inputs"));

  // Parameters for inputs substitution
  parameter Modelica.Units.SI.Temperature T_sec=323.15
    "Temperature set point for the secondary side"
    annotation (Dialog(group="Fixed control inputs"));
  parameter Modelica.Units.SI.MassFlowRate m_flow_sec=0.358
    "Mass flow set point for the secondary side"
    annotation (Dialog(group="Fixed control inputs"));

  // Dynamic parameters for CV in PrescribedSecondarySide
  extends ProsNet.Prosumers.SecondarySides.BaseClasses.ControlVolumeDynParam;
  // PumpsPairPartial dynamic parameters
  extends ProsNet.Prosumers.SecondarySides.BaseClasses.PumpsPairDynParam;

  parameter Modelica.Units.SI.Time tau_temSen=1
    "Time constant for the temperature sensors"
    annotation (Dialog(tab="Advanced", group="Temperature sensors dynamics"));
  parameter Modelica.Blocks.Types.Init init_temSen=Modelica.Blocks.Types.Init.SteadyState "Type of initialization"
  annotation(Dialog(tab="Advanced", group="Temperature sensors dynamics"));

  SecondarySides.PrescribedSecondarySide secSide(
    redeclare final package Medium = Medium2,
    final dp_nominal=dp2_nominal,
    final m_flow_nominal_cv=m_flow_nominal_2,
    final dp_nominal_cv=0,
    final tau_cv=tau_cv,
    final energyDynamics_cv = energyDynamics_cv,
    final T_start_cv=T_start_cv,
    final energyDynamics_pumpsSec = energyDynamics_pumpsSec,
    final tau_pumpsSec = tau_pumpsSec,
    final use_inputFilter_pumpsSec = use_inputFilter_pumpsSec,
    final riseTime_pumpsSec=riseTime_pumpsSec,
    final m_flow_start_pumpsSec=m_flow_start_pumpsSec,
    final y_start_pumpsSec = y_start_pumpsSec) annotation (Placement(transformation(extent={{-6,-86},{14,-66}})));

  // Temperature sensors

  // Conditional input connectors
  Modelica.Blocks.Interfaces.RealInput m_flow_sec_set(unit="kg/s", displayUnit="kg/s")
   if use_m_flow_set_in "Mass flow rate on the secondary side" annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={20,120})));
  Modelica.Blocks.Interfaces.RealInput T_sec_set(unit="K", displayUnit="degC")
   if use_T_set_in "Temperature on the secondary side" annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={-20,120})));

  // Simulation results as internal variables
public
  Modelica.Units.SI.Temperature T_prim_hot(displayUnit="degC")
    "Temperature in the hot port on the primary side";

  Modelica.Units.SI.Temperature T_prim_cold(displayUnit="degC")
    "Temperature in the cold port on the primary side";

  Modelica.Units.SI.Temperature T_sec_hot(displayUnit="degC")
    "Temperature in the hot port on the secondary side";

  Modelica.Units.SI.Temperature T_sec_cold(displayUnit="degC")
    "Temperature in the cold port on the secondary side";

  Modelica.Units.SI.HeatFlowRate Q_transf
    "Heat flow rate in the nextwork direction";

    Real dotV_prim(final unit = "l/min") "Volume flow rate on the primary side";

    Real dotV_sec(final unit = "l/min") "Volume flow rate on the secondary side";

  Modelica.Units.SI.Pressure Delta_p_prim(displayUnit="Pa")
    "Pressure drop between cold and hot ports";

  // Internal input connectors
protected
  Modelica.Blocks.Interfaces.RealInput m_flow_sec_set_internal annotation (
      Placement(visible=false,transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={20,120}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={32,120})));
  Modelica.Blocks.Interfaces.RealInput T_sec_set_internal annotation (Placement(
        visible = false,transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={-20,120}),iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={-16,120})));

  Medium1.Density rho_a_inflow "Density of inflowing fluid at port_a";
  Medium1.Density rho_b_outflow "Density of outflowing fluid at port_b";
  Medium1.Density average_rho_a_b "Average density of the flow from port_a to port_b";

  Medium1.Density rho_b_inflow "Density of inflowing fluid at port_b";
  Medium1.Density rho_a_outflow "Density of outflowing fluid at port_a";
  Medium1.Density average_rho_b_a "Average density of the flow from port_b to port_a";

  Medium1.Density density_average "Average density inside the control volume";

  Fluid.Sensors.TemperatureTwoPort temSecHot(
    redeclare final package Medium = Medium2,
    m_flow_nominal=m_flow_nominal_2,
    tau=tau_temSen,
    initType=init_temSen) "Hot port temperature sensor on the secondary side"
    annotation (Placement(transformation(extent={{30,-86},{46,-66}})));

   Fluid.Sensors.TemperatureTwoPort temSecCold(
    redeclare final package Medium = Medium2,
    m_flow_nominal=m_flow_nominal_2,
    tau=tau_temSen,
    initType=init_temSen) "Cold port temperature sensor on the secondary side"
    annotation (Placement(transformation(extent={{-62,-86},{-82,-66}})));

equation

  T_prim_hot = temPriHot.T;
  T_prim_cold = temPriCold.T;
  T_sec_hot = temSecHot.T;
  T_sec_cold = temSecCold.T;

  Q_transf = priSide.HEX.Q1_flow;

  Delta_p_prim = port_a.p - port_b.p;

  // Average density in between hot and cold ports on the primary side
  rho_a_inflow = Medium1.density(Medium.setState_phX(port_a.p,
 inStream(port_a.h_outflow),
 inStream(port_a.Xi_outflow)));
  rho_b_outflow = Medium1.density(Medium.setState_phX(port_b.p,
 port_b.h_outflow,
 port_b.Xi_outflow));
  average_rho_a_b = (rho_a_inflow + rho_b_outflow)/2;

  rho_b_inflow = Medium1.density(Medium.setState_phX(port_b.p,
 inStream(port_b.h_outflow),
 inStream(port_b.Xi_outflow)));
  rho_a_outflow = Medium1.density(Medium.setState_phX(port_a.p,
 port_a.h_outflow,
 port_a.Xi_outflow));
  average_rho_b_a = (rho_b_inflow + rho_a_outflow)/2;

  m1_flow_small = priSide.m1_flow_small;

  density_average = Modelica.Fluid.Utilities.regStep(port_a.m_flow, average_rho_a_b, average_rho_b_a, m1_flow_small);

  dotV_prim = (1000*port_a.m_flow/density_average)*60;

  // ADD average density in between hot and cold ports on the secondary side

  dotV_sec = (1000*priSide.port_b2.m_flow/density_average)*60;

  // Substitution for internal inputs
  if not use_T_set_in then
    T_sec_set_internal = T_sec;
  end if;
  if not use_m_flow_set_in then
    m_flow_sec_set_internal = m_flow_sec;
  end if;

  // Connect internal inputs with submodels
  connect(secSide.T_set, T_sec_set_internal)
    annotation (Line(points={{0,-64},{0,18},{-20,18},{-20,120}}, color={0,0,127}));
  connect(secSide.m_flow_set, m_flow_sec_set_internal)
  annotation (Line(points={{8,-64},{8,2},{20,2},{20,120}}, color={0,0,127}));

  // Connect internal inputs with condition inputs
  connect(m_flow_sec_set,m_flow_sec_set_internal);
  connect(T_sec_set,T_sec_set_internal);

  connect(secSide.mu, mu_set_internal) annotation (Line(points={{-8,-61},{-54,-61},{-54,
          24},{-80,24},{-80,120}}, color={255,127,0}));
  connect(secSide.pi, pi_set_internal) annotation (Line(points={{-8,-64.8},{-60,-64.8},{
          -60,18},{-116,18},{-116,120}}, color={255,127,0}));

  // Temperature sensors
  connect(secSide.port_b, temSecHot.port_a)
    annotation (Line(points={{14,-76},{30,-76}}, color={0,127,255}));
  connect(priSide.port_b2, temSecCold.port_b) annotation (Line(points={{28,-44},{-16,-44},
          {-16,-50},{-94,-50},{-94,-76},{-82,-76}}, color={0,127,255}));
  connect(temSecCold.port_a, secSide.port_a)
    annotation (Line(points={{-62,-76},{-6,-76}}, color={0,127,255}));

  connect(temSecHot.port_b, priSide.port_a2) annotation (Line(points={{46,-76},{
          52,-76},{52,-44},{48,-44}}, color={0,127,255}));
  annotation (defaultComponentName="pros", Icon(coordinateSystem(preserveAspectRatio=true), graphics={
                     Rectangle(extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),                                                                                           Rectangle(origin={36,
              2},                                                                                                                                                 fillColor = {255, 0, 0},
            fillPattern =                                                                                                                                                                                FillPattern.Solid, extent={{
              -36,32},{36,-32}}),                                                                                                                                                                                                        Rectangle(origin={
              -36,2},                                                                                                                                                                                                        fillColor = {85, 0, 255},
            fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent={{
              -36,32},{36,-32}})}),                              Diagram(
        coordinateSystem(preserveAspectRatio=true)),
    Documentation(info="<html>
<p>This is a prosumer model with the <a href=\"modelica://ProsNet.Prosumers.SecondarySides.PrescribedSecondarySide\">PrescribedSecondarySide</a> model that defines inlet temperature and mass flow rate on the secondary side according to prescribed inputs.</p>
<p>The prosumer model essentially consists of two parts: primary and secondary side. The <a href=\"modelica://ProsNet.Prosuers.BaseClasses.PrimarySide\">PrimarySide</a> model represents elements for bidirectional heat flow between the prosumer and the network and the unidirectional mass flow rate that is associated with this heat transfer. Port_a and port_b are called hot and cold ports respectively, they are next to the red and blue squares. It is not allowed to connect a hot port with a cold one directly.</p>
<p>Individual parameters for the primary and secondary are collected in the General and Dynamics tabs. To avoid repetition, the details on the parametrization of the respective sides can be found directly in the <a href=\"modelica://ProsNet.Prosumers.BaseClasses.PrimarySide\">PrimarySide</a>, <a href=\"modelica://ProsNet.Prosumers.BaseClasses.PrimarySidePartial\">PrimarySidePartial</a>, and <a href=\"modelica://ProsNet.Prosumers.SecondarySides.PrescribedSecondarySide\">PrescribedSecondarySide</a> models. Local hydraulic losses must be defined for the nominal mass flow rate of the heat exchanger on the primary side. Note that, these losses are intended to represent pressure drop due to pipes, bends, junctions, etc. within the prosumer, and has no connection to the pressure losses due to the heat exchanger, control, or check valve.</p>
<p>An important feature of the model is that the user can choose whether he or she wants to read input values from the input connectors or provide them as parameters. This can be done by toggling a respective input in the Switching input section in the General tab.</p>
<p>The mass flow rate and the temperatures at hot and cold ports will be shown at the prosumer&rsquo;s icon interactively (only valid for Dymola). Positive mass flow rate corresponds to consumption mode, negative for production. Note that, the indicators show valid results only for steady-state simulations, for dynamic simulations these values display initial values.</p>
</html>"));
end ProsumerIdeal;
