within ProsNet.Prosumers.BaseClasses;
partial model PrimarySidePartial

  replaceable package Medium1 = ProsNet.Media.Water;
  replaceable package Medium2 = ProsNet.Media.Water;

  extends ProsNet.Fluid.Interfaces.PartialTwoPort(redeclare final package Medium = Medium1);

  /* Parameters */
  // Conditional inputs
  parameter Boolean use_mu_set_in = false "Get operating mode from the input connector"
    annotation(Dialog(group="Conditional control inputs"));
  parameter Boolean use_pi_set_in = false "Get participation mode from the input connector"
    annotation(Dialog(group="Conditional control inputs"));
  parameter Boolean use_kappa_set_in = false "Get the main valve's opening from the input connector"
    annotation(Dialog(group="Conditional control inputs"));
  parameter Boolean use_u_set_in = false "Get the main pump's velocity from the input connector"
    annotation(Dialog(group="Conditional control inputs"));

  // Parameters for inputs substitution
  parameter ProsNet.Prosumers.Types.OperationMode mode=ProsNet.Prosumers.Types.OperationMode.Production "Mode of operation"
   annotation(Dialog(group="Fixed control inputs"));
  parameter Boolean participation=true "Participation"
   annotation(Dialog(group="Fixed control inputs"));
  parameter Real kappa(min=0, max=1)=1 "Normalized flow coefficient for control valve"
    annotation(Dialog(group="Fixed control inputs"));
  parameter Real u(min=0, max=1)=1 "Normalized feed-in pump rotational velocity"
    annotation(Dialog(group="Fixed control inputs"));

  // General primary side parameters
  extends ProsNet.Prosumers.BaseClasses.PrimarySideParameters;

  parameter Modelica.Units.SI.Time tau_temSen=1
    "Time constant for the temperature sensors"
    annotation (Dialog(tab="Advanced", group="Temperature sensors dynamics"));
  parameter Modelica.Blocks.Types.Init init_temSen=Modelica.Blocks.Types.Init.SteadyState "Type of initialization"
  annotation(Dialog(tab="Advanced", group="Temperature sensors dynamics"));

  PrimarySide priSide(
    redeclare final package Medium1 = Medium1,
    redeclare final package Medium2 = Medium2,
    Q_flow_nominal=Q_flow_nominal,
    T_a1_nominal=T_a1_nominal,
    T_a2_nominal=T_a2_nominal,
    m_flow_nominal_1=m_flow_nominal_1,
    dp1_nominal=dp1_nominal,
    m_flow_nominal_2=m_flow_nominal_2,
    dp2_nominal=dp2_nominal,
    energyDynamics_feedPump=energyDynamics_feedPump,
    tau_feedPump=tau_feedPump,
    r_nominal=r_nominal,
    n=n,
    feedinPer=feedinPer,
    use_inputFilter_feedPump=use_inputFilter_feedPump,
    riseTime_feedPump=riseTime_feedPump,
    init_feedPump=init_feedPump,
    y_start_feedPump=y_start_feedPump,
    Kv_conVal=Kv_conVal,
    l_conVal=l_conVal,
    use_inputFilter_conVal=use_inputFilter_conVal,
    riseTime_conVal=riseTime_conVal,
    init_conVal=init_conVal,
    y_start_conVal=y_start_conVal,
    Kv_cheVal=Kv_cheVal,
    l_cheVal=l_cheVal)
    annotation (Placement(transformation(extent={{28,-48},{48,-28}})));

  Fluid.Sources.Boundary_pT bou(redeclare final package Medium = Medium2, nPorts=1)
    annotation (Placement(transformation(extent={{74,-68},{62,-56}})));

            Fluid.Sensors.TemperatureTwoPort temPriHot(redeclare final package Medium = Medium1,
    m_flow_nominal=m_flow_nominal_1,
    tau=tau_temSen,
    initType=init_temSen) "Hot port temperature sensor"
    annotation (Placement(transformation(extent={{74,-10},{90,10}})));

            Fluid.Sensors.TemperatureTwoPort temPriCold(redeclare final package
                                                                                Medium = Medium1,
    m_flow_nominal=m_flow_nominal_1,
    tau=tau_temSen,
    initType=init_temSen) "Cold port temperature sensor"
    annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));

  // Activates conditional inputs
  Modelica.Blocks.Interfaces.IntegerInput mu_set if use_mu_set_in
    "Operating mode" annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={-80,120}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={-80,120})));
  Modelica.Blocks.Interfaces.IntegerInput pi_set if use_pi_set_in
    "Participation" annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={-116,118}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={-126,120})));
  Modelica.Blocks.Interfaces.RealInput kappa_set if use_kappa_set_in
    "Normalized flow coefficient for control valve"
     annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={80,120}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={136,120})));
  Modelica.Blocks.Interfaces.RealInput u_set if use_u_set_in
    "Normalized velocity of feed-in pump"
      annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={118,120}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={90,120})));

  ProsNet.Controls.Linearizer lin(redeclare final ProsNet.Controls.Data.Linearizer.EqualPercentage cha)
   annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={80,70})));

protected
  Modelica.Blocks.Interfaces.IntegerInput pi_set_internal "Participation"
    annotation (Placement(
      visible=false,
      transformation(
        origin={-116,120},
        extent={{-20,-20},{20,20}},
        rotation=-90),
      iconTransformation(
        origin={-120,120},
        extent={{-20,-20},{20,20}},
        rotation=-90)));
  Modelica.Blocks.Interfaces.IntegerInput mu_set_internal "Operating mode"
    annotation (Placement(
      visible=false,
      transformation(
        origin={-80,120},
        extent={{-20,-20},{20,20}},
        rotation=-90),
      iconTransformation(
        origin={-70,120},
        extent={{-20,-20},{20,20}},
        rotation=-90)));
  Modelica.Blocks.Interfaces.RealInput u_set_internal
   "Normalized main pump's velocity (internal)"
    annotation (Placement(
      visible=false,
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={118,120})));
  Modelica.Blocks.Interfaces.RealInput kappa_set_internal
    "Normalized linearized flow coefficient (internal)"
       annotation (Placement(
         visible=false,
         transformation(
         extent={{-20,-20},{20,20}},
         rotation=-90,
         origin={80,120})));

equation

  // Substitution for operating mode and participation if they are set as parameters
  if not use_mu_set_in then
    mu_set_internal = if mode == ProsNet.Prosumers.Types.OperationMode.Production then 1 else -1;
  end if;
  if not use_pi_set_in then
    pi_set_internal = if participation then 1 else 0;
  end if;

  // Substitution for control valve flow coefficient
  if not use_kappa_set_in then
    kappa_set_internal = kappa;
  end if;
  // Substitution for normalized pump speed
  if not use_u_set_in then
    u_set_internal = u;
  end if;

  // Connect conditional and internal inputs
  connect(mu_set,mu_set_internal);
  connect(pi_set,pi_set_internal);
  connect(kappa_set, kappa_set_internal);
  connect(u_set, u_set_internal);

  connect(temPriCold.port_a, port_a)
    annotation (Line(points={{-90,0},{-100,0}}, color={0,127,255}));
  connect(temPriHot.port_b, port_b)
    annotation (Line(points={{90,0},{100,0}}, color={0,127,255}));
  connect(priSide.port_a2, bou.ports[1]) annotation (Line(points={{48,-44},{56,-44},{56,
          -62},{62,-62}}, color={0,127,255}));
  connect(priSide.pi, pi_set_internal) annotation (Line(points={{26,-26},{-60,-26},{-60,
          18},{-116,18},{-116,120}}, color={255,127,0}));
  connect(priSide.mu, mu_set_internal) annotation (Line(points={{26,-22.2},{-54,-22.2},{
          -54,24},{-80,24},{-80,120}}, color={255,127,0}));
  connect(lin.kappa, kappa_set_internal)
    annotation (Line(points={{80,82},{80,120}}, color={0,0,127}));
  connect(lin.op, priSide.conVal_op_set) annotation (Line(points={{80,59},{80,28},{48,28},
          {48,-4},{34,-4},{34,-26}}, color={0,0,127}));
  connect(priSide.feedPump_y_set, u_set_internal) annotation (Line(points={{42,-26},{42,
          -14},{58,-14},{58,20},{118,20},{118,120}}, color={0,0,127}));

  connect(priSide.port_b1, temPriHot.port_a) annotation (Line(points={{48,-32},{
          64,-32},{64,0},{74,0}}, color={0,127,255}));
  connect(temPriCold.port_b, priSide.port_a1) annotation (Line(points={{-70,0},{
          -64,0},{-64,-32},{28,-32}}, color={0,127,255}));
annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-30,8},{72,-82}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-10,-14},{54,-34}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-10,-34},{52,-16}},
                                        color={28,108,200}),
        Ellipse(
          extent={{-10,-52},{14,-76}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{34,-52},{54,-52},{44,-64},{34,-52}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-4,-58},{8,-58},{2,-52},{-4,-58}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Line(points={{6,-14},{6,0}},     color={28,108,200}),
        Line(points={{36,-14},{36,0}}, color={28,108,200}),
        Line(points={{6,-34},{6,-46}},   color={28,108,200}),
        Line(points={{36,-34},{36,-46}},
                                       color={28,108,200}),
        Line(points={{32,-64},{44,-64},{38,-60}}, color={28,108,200}),
        Line(points={{32,-64},{44,-64},{38,-68}}, color={28,108,200}),
        Polygon(
          points={{34,-76},{54,-76},{44,-64},{34,-76}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-104,118},{-224,68}},
          lineColor={0,0,0},
          textString="T_c"),                                                                              Text(
          textString=DynamicSelect("", String(temPriCold.T-273.15, format=".1f")),
          extent={{-96,54},{-216,4}},
          lineColor={0,0,0}),
        Text(
          extent={{242,118},{122,68}},
          lineColor={0,0,0},
          textString="T_h"),                                                                              Text(
          textString=DynamicSelect("", String(temPriHot.T-273.15, format=".1f")),
          extent={{248,60},{128,10}},
          lineColor={0,0,0}),
        Text(
          extent={{248,-14},{128,-64}},
          lineColor={0,0,0},
          textString="m_flow"),                                                                           Text(
          textString=DynamicSelect("", String(port_b.m_flow, format=".2f")),
          extent={{226,-70},{106,-120}},
          lineColor={0,0,0})}),             Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end PrimarySidePartial;
