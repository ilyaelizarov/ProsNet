within ProsNet.Under_Development.new_prosumer_models;
model Test_heat_transfer_station_production
  Modelica.Blocks.Sources.RealExpression T_house(y=273.15 + 70)
    annotation (Placement(transformation(extent={{-92,74},{-72,94}})));
  Modelica.Blocks.Sources.IntegerExpression pi(y=1)
    annotation (Placement(transformation(extent={{-92,44},{-72,64}})));
  Modelica.Blocks.Sources.IntegerExpression mu(y=1)
    annotation (Placement(transformation(extent={{-92,30},{-72,50}})));
  Modelica.Blocks.Sources.RealExpression u_pump(y=0.5)
    annotation (Placement(transformation(extent={{-92,12},{-72,32}})));
  Modelica.Blocks.Sources.RealExpression kappa(y=0.8)
    annotation (Placement(transformation(extent={{-92,-2},{-72,18}})));
  Modelica.Blocks.Sources.RealExpression flow_house(y=5)
    annotation (Placement(transformation(extent={{-92,62},{-72,82}})));
  Modelica.Blocks.Sources.Ramp ramp(
    height=1,
    duration=600,
    offset=0,
    startTime=200)
    annotation (Placement(transformation(extent={{-92,-32},{-72,-12}})));
  heat_transfer_station heat_transfer_station1(redeclare
      Fluid.Pumps.Data.Pumps.QuadraticCharacteristic feedinPer,
    energyDynamics_feedPump=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    use_inputFilter_feedPump=true,
    init_feedPump=Modelica.Blocks.Types.Init.InitialOutput,
    use_inputFilter_conVal=true,
    init_conVal=Modelica.Blocks.Types.Init.InitialOutput,
      ambient_temperature=system.T_ambient,
    energyDynamics_pumpsSec=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    use_inputFilter_pumpsSec=true)
    annotation (Placement(transformation(extent={{-8,40},{22,76}})));

  Modelica.Fluid.Vessels.ClosedVolume volume(
    T_start=318.15,
    use_portsData=false,                     V=1, nPorts=2,
  redeclare final package Medium = Media.Water)
    annotation (Placement(transformation(extent={{2,-24},{22,-44}})));
  inner Modelica.Fluid.System system(T_ambient=285.15)
    annotation (Placement(transformation(extent={{60,62},{80,82}})));
  Fluid.Valves.TwoWayEqualPercentage valve_for_test(
    m_flow_nominal=30.074213*0.001/60,
    redeclare final package Medium = Media.Water,
    final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
    final Kv=2.5,
    final use_inputFilter=true,
    final riseTime=5,
    final init=Modelica.Blocks.Types.Init.InitialOutput,
    final y_start=0,
    final l=2e-3) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=-90,
        origin={36,-4})));
  Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare final package Medium =
        Media.Water)
    annotation (Placement(transformation(extent={{106,2},{86,22}})));
equation
  connect(T_house.y, heat_transfer_station1.T_sec_in_set) annotation (Line(
        points={{-71,84},{-14,84},{-14,72},{-8,72}}, color={0,0,127}));
  connect(flow_house.y, heat_transfer_station1.V_dot_sec_set) annotation (Line(
        points={{-71,72},{-16,72},{-16,68},{-8,68}}, color={0,0,127}));
  connect(pi.y, heat_transfer_station1.pi) annotation (Line(points={{-71,54},{-16,
          54},{-16,62},{-8,62}}, color={255,127,0}));
  connect(mu.y, heat_transfer_station1.mu) annotation (Line(points={{-71,40},{-14,
          40},{-14,58},{-8,58}}, color={255,127,0}));
  connect(u_pump.y, heat_transfer_station1.u_set) annotation (Line(points={{-71,
          22},{-66,22},{-66,52},{-60,52},{-60,54},{-8,54}}, color={0,0,127}));
  connect(kappa.y, heat_transfer_station1.kappa_set) annotation (Line(points={{-71,
          8},{-58,8},{-58,50},{-8,50}}, color={0,0,127}));
  connect(heat_transfer_station1.hot_prim, volume.ports[1])
    annotation (Line(points={{-3.5,39.8},{-3.5,-24},{11,-24}},
                                                         color={0,127,255}));
  connect(valve_for_test.port_b, heat_transfer_station1.cold_prim) annotation (
      Line(points={{36,6},{36,34},{17.5,34},{17.5,40}},
                                                      color={0,127,255}));
  connect(valve_for_test.port_a, volume.ports[2])
    annotation (Line(points={{36,-14},{36,-24},{13,-24}}, color={0,127,255}));
  connect(ramp.y, valve_for_test.y) annotation (Line(points={{-71,-22},{0,-22},{
          0,-4},{24,-4}}, color={0,0,127}));
  connect(bou.ports[1], valve_for_test.port_b)
    annotation (Line(points={{86,12},{36,12},{36,6}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(lineColor = {75,138,73},
                fillColor={255,255,255},
                fillPattern = FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(lineColor = {0,0,255},
                fillColor = {75,138,73},
                pattern = LinePattern.None,
                fillPattern = FillPattern.Solid,
                points={{-36,60},{64,0},{-36,-60},{-36,60}})}),  Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=1000,
      Interval=1,
      __Dymola_Algorithm="Dassl"));
end Test_heat_transfer_station_production;
