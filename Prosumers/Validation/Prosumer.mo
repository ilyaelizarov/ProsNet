within ProsNet.Prosumers.Validation;
model Prosumer
  extends Modelica.Icons.Example;

  Fluid.Sources.Boundary_pT bou1(
    redeclare package Medium = Media.Water,
    T=278.15,                               nPorts=5)
    annotation (Placement(transformation(extent={{-88,6},{-68,26}})));
  Fluid.Sources.Boundary_pT bou2(
    redeclare package Medium = Media.Water, nPorts=3)
    annotation (Placement(transformation(extent={{86,4},{66,24}})));
  ProsumerIdeal prodSteadyState(
    u=1,
    T_sec=323.15,
    m_flow_sec=0.358)
    annotation (Placement(transformation(extent={{-12,26},{8,46}})));
  ProsumerIdeal prodFixedInitial(
    participation=true,
    u=0.5,
    T_sec=323.15,
    m_flow_sec=0.358,
    energyDynamics_cv=Modelica.Fluid.Types.Dynamics.FixedInitial)
    annotation (Placement(transformation(extent={{-12,-12},{8,8}})));
  Fluid.Sources.Boundary_pT bou4(
    redeclare package Medium = Media.Water,
    p=340000,
    T=343.15,
    nPorts=2)
    annotation (Placement(transformation(extent={{86,-52},{66,-32}})));
  ProsumerIdeal consSteadyState(
    mode=ProsNet.Prosumers.Types.OperationMode.Consumption,
    T_sec=303.15,
    m_flow_sec=0.358,
    energyDynamics_cv=Modelica.Fluid.Types.Dynamics.SteadyState)
    annotation (Placement(transformation(extent={{-12,-52},{8,-32}})));
  ProsumerIdeal consValveFilter(
    mode=ProsNet.Prosumers.Types.OperationMode.Consumption,
    use_inputFilter_conVal=true,
    riseTime_conVal=30,
    init_conVal=Modelica.Blocks.Types.Init.InitialOutput,
    y_start_conVal=0.98,
    T_sec=303.15,
    m_flow_sec=0.358)
    annotation (Placement(transformation(extent={{-12,-86},{8,-66}})));
  ProsumerIdeal prodSecPumpDyn(
    mode=ProsNet.Prosumers.Types.OperationMode.Production,
    T_sec=343.15,
    m_flow_sec=0.358,
    energyDynamics_cv=Modelica.Fluid.Types.Dynamics.FixedInitial,
    tau_cv=100,
    T_start_cv=278.15,
    use_inputFilter_pumpsSec=false,
    riseTime_pumpsSec=30,
    m_flow_start_pumpsSec=1,
    y_start_pumpsSec=1)
    annotation (Placement(transformation(extent={{-12,68},{8,88}})));
equation
  connect(bou1.ports[1], prodSteadyState.port_a) annotation (Line(points={{-68,
          14.4},{-46,14.4},{-46,36},{-12,36}},
                                       color={0,127,255}));
  connect(bou2.ports[1], prodSteadyState.port_b) annotation (Line(points={{66,
          12.6667},{50,12.6667},{50,36},{8,36}},
                                    color={0,127,255}));
  connect(prodFixedInitial.port_b, bou2.ports[2]) annotation (Line(points={{8,-2},{
          50,-2},{50,14},{66,14}},  color={0,127,255}));
  connect(prodFixedInitial.port_a, bou1.ports[2]) annotation (Line(points={{-12,-2},
          {-46,-2},{-46,15.2},{-68,15.2}}, color={0,127,255}));
  connect(consSteadyState.port_b, bou4.ports[1])
    annotation (Line(points={{8,-42},{38,-42},{38,-43},{66,-43}},
                                                           color={0,127,255}));
  connect(consSteadyState.port_a, bou1.ports[3]) annotation (Line(points={{-12,-42},
          {-46,-42},{-46,14},{-68,14},{-68,16}},
                                               color={0,127,255}));
  connect(consValveFilter.port_a, bou1.ports[4]) annotation (Line(points={{-12,-76},
          {-46,-76},{-46,14},{-68,14},{-68,16.8}},
                                                 color={0,127,255}));
  connect(consValveFilter.port_b, bou4.ports[2]) annotation (Line(points={{8,-76},
          {52,-76},{52,-41},{66,-41}},
                                   color={0,127,255}));
  connect(prodSecPumpDyn.port_b, bou2.ports[3]) annotation (Line(points={{8,78},{
          50,78},{50,20},{66,20},{66,15.3333}},  color={0,127,255}));
  connect(prodSecPumpDyn.port_a, bou1.ports[5]) annotation (Line(points={{-12,78},
          {-46,78},{-46,34},{-68,34},{-68,17.6}},     color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=100, __Dymola_Algorithm="Dassl"));
end Prosumer;
