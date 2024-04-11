within ProsNet.Under_Development.Tests;
model Test_pipe_model
  Modelica.Fluid.Sensors.VolumeFlowRate volumeFlowRate(redeclare package Medium =
        ProsNet.Media.Water)
    annotation (Placement(transformation(extent={{-20,32},{0,52}})));
  Modelica.Fluid.Sensors.RelativePressure relativePressure(redeclare package
      Medium = ProsNet.Media.Water)
    annotation (Placement(transformation(extent={{20,64},{40,84}})));
  Modelica.Fluid.Sensors.RelativeTemperature relativeTemperature(redeclare
      package Medium = ProsNet.Media.Water)
    annotation (Placement(transformation(extent={{20,4},{40,-16}})));
  Modelica.Fluid.Sources.MassFlowSource_T mass_source(
    redeclare package Medium = ProsNet.Media.Water,
    use_m_flow_in=true,
    use_T_in=true,
    m_flow=1,
    T(displayUnit="K"),
    nPorts=1) annotation (Placement(transformation(extent={{-94,32},{-74,52}})));
  inner Modelica.Fluid.System system(
    T_ambient=285.15,
    allowFlowReversal=true,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial)
    annotation (Placement(transformation(extent={{132,76},{152,96}})));
  Modelica.Fluid.Sources.Boundary_pT bou(redeclare package Medium =
        ProsNet.Media.Water, nPorts=1)
    annotation (Placement(transformation(extent={{104,32},{84,52}})));
  ProsNet.Under_Development.Controller_PID_based.auxiliary.TimeTable_noInterp volume_flow(table=[0,
        1; 10,5; 20,10; 30,11.4; 40,15; 50,20; 60,1; 70,5; 80,10; 90,11.4; 100,
        15; 110,20; 120,1; 130,5; 140,10; 150,11.4; 160,15; 170,20; 7470,20;
        7480,0; 10980,20; 17580,20], timeScale=1)
    annotation (Placement(transformation(extent={{-182,50},{-162,70}})));
  ProsNet.Under_Development.Controller_PID_based.auxiliary.TimeTable_noInterp temperatures(
    table=[0,30; 10,30; 20,30; 30,30; 40,30; 50,30; 60,60; 70,60; 80,60; 90,60;
        100,60; 110,60; 120,90; 130,90; 140,90; 150,90; 160,90; 170,90; 180,90;
        7480,90; 10980,90; 17580,90],
    timeScale=1,
    y(unit="K"))
    annotation (Placement(transformation(extent={{-182,16},{-162,36}})));
  Modelica.Blocks.Math.Gain gain(k=1/60.266)
    annotation (Placement(transformation(extent={{-138,50},{-118,70}})));
  Modelica.Blocks.Math.UnitConversions.From_degC from_degC
    annotation (Placement(transformation(extent={{-138,16},{-118,36}})));
  ProsNet.Fluid.Sensors.TemperatureTwoPort Tem_in(
    redeclare package Medium = ProsNet.Media.Water,
    m_flow_nominal=1/6,
    tau=0) annotation (Placement(transformation(extent={{-54,32},{-34,52}})));
  ProsNet.Fluid.Sensors.TemperatureTwoPort Tem_out(
    redeclare package Medium = ProsNet.Media.Water,
    m_flow_nominal=1/6,
    tau=0) annotation (Placement(transformation(extent={{54,32},{74,52}})));

  Real DeltaT;
  Modelica.Fluid.Sensors.VolumeFlowRate volumeFlowRate1(redeclare package
      Medium = ProsNet.Media.Water)
    annotation (Placement(transformation(extent={{-18,-80},{2,-60}})));
  Modelica.Fluid.Sensors.RelativePressure relativePressure1(redeclare package
      Medium = ProsNet.Media.Water)
    annotation (Placement(transformation(extent={{22,-48},{42,-28}})));
  Modelica.Fluid.Sensors.RelativeTemperature relativeTemperature1(redeclare
      package Medium = ProsNet.Media.Water)
    annotation (Placement(transformation(extent={{22,-108},{42,-128}})));
  Modelica.Fluid.Sources.Boundary_pT bou1(redeclare package Medium =
        ProsNet.Media.Water, nPorts=1)
    annotation (Placement(transformation(extent={{106,-80},{86,-60}})));
  ProsNet.Fluid.Sensors.TemperatureTwoPort Tem_in1(
    redeclare package Medium = ProsNet.Media.Water,
    m_flow_nominal=1/6,
    tau=0) annotation (Placement(transformation(extent={{-52,-80},{-32,-60}})));
  ProsNet.Fluid.Sensors.TemperatureTwoPort Tem_out1(
    redeclare package Medium = ProsNet.Media.Water,
    m_flow_nominal=1/6,
    tau=0) annotation (Placement(transformation(extent={{56,-80},{76,-60}})));
  ProsNet.Fluid.Pipes.InsulatedPipe_plug pipe_new(
    allowFlowReversal=true,
    length=1000,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
    annotation (Placement(transformation(extent={{18,-80},{38,-60}})));
  Modelica.Fluid.Sources.MassFlowSource_T mass_source1(
    redeclare package Medium = ProsNet.Media.Water,
    use_m_flow_in=true,
    use_T_in=true,
    m_flow=1,
    T(displayUnit="K"),
    nPorts=1) annotation (Placement(transformation(extent={{-88,-80},{-68,-60}})));
  ProsNet.Fluid.Pipes.InsulatedPipe pipe(
    allowFlowReversal=true,
    length=1000,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial)
    annotation (Placement(transformation(extent={{18,30},{38,50}})));
equation
  connect(volume_flow.y, gain.u) annotation (Line(
      points={{-161,60},{-161,60},{-142,60},{-140,60}},
      color={0,0,127},
      smooth=Smooth.Bezier));
  connect(temperatures.y, from_degC.u) annotation (Line(
      points={{-161,26},{-161,26},{-144,26},{-140,26}},
      color={0,0,127},
      smooth=Smooth.Bezier));
  connect(mass_source.ports[1], Tem_in.port_a) annotation (Line(
      points={{-74,42},{-54,42}},
      color={0,127,255},
      smooth=Smooth.Bezier));
  connect(Tem_in.port_b, volumeFlowRate.port_a) annotation (Line(
      points={{-34,42},{-20,42}},
      color={0,127,255},
      smooth=Smooth.Bezier));
  connect(Tem_out.port_b, bou.ports[1]) annotation (Line(
      points={{74,42},{84,42}},
      color={0,127,255},
      smooth=Smooth.Bezier));

  DeltaT = Tem_out.T - Tem_in.T;
  connect(volumeFlowRate.port_b, relativePressure.port_a) annotation (Line(
        points={{0,42},{8,42},{8,74},{20,74}}, color={0,127,255}));
  connect(relativePressure.port_b, Tem_out.port_a) annotation (Line(points={{40,
          74},{48,74},{48,42},{54,42}}, color={0,127,255}));
  connect(relativeTemperature.port_b, Tem_out.port_a) annotation (Line(points={
          {40,-6},{46,-6},{46,42},{54,42}}, color={0,127,255}));
  connect(relativeTemperature.port_a, volumeFlowRate.port_b) annotation (Line(
        points={{20,-6},{10,-6},{10,42},{0,42}}, color={0,127,255}));
  connect(Tem_in1.port_b, volumeFlowRate1.port_a) annotation (Line(
      points={{-32,-70},{-18,-70}},
      color={0,127,255},
      smooth=Smooth.Bezier));
  connect(Tem_out1.port_b, bou1.ports[1]) annotation (Line(
      points={{76,-70},{86,-70}},
      color={0,127,255},
      smooth=Smooth.Bezier));
  connect(volumeFlowRate1.port_b, relativePressure1.port_a) annotation (Line(
        points={{2,-70},{10,-70},{10,-38},{22,-38}}, color={0,127,255}));
  connect(relativePressure1.port_b, Tem_out1.port_a) annotation (Line(points={{
          42,-38},{50,-38},{50,-70},{56,-70}}, color={0,127,255}));
  connect(relativeTemperature1.port_b, Tem_out1.port_a) annotation (Line(points=
         {{42,-118},{50,-118},{50,-70},{56,-70}}, color={0,127,255}));
  connect(relativeTemperature1.port_a, volumeFlowRate1.port_b) annotation (Line(
        points={{22,-118},{10,-118},{10,-70},{2,-70}}, color={0,127,255}));
  connect(volumeFlowRate1.port_b, pipe_new.port_a)
    annotation (Line(points={{2,-70},{18,-70}}, color={0,127,255}));
  connect(pipe_new.port_b, Tem_out1.port_a)
    annotation (Line(points={{38,-70},{56,-70}}, color={0,127,255}));
  connect(mass_source1.ports[1], Tem_in1.port_a)
    annotation (Line(points={{-68,-70},{-52,-70}}, color={0,127,255}));
  connect(gain.y, mass_source.m_flow_in)
    annotation (Line(points={{-117,60},{-94,60},{-94,50}}, color={0,0,127}));
  connect(gain.y, mass_source1.m_flow_in) annotation (Line(points={{-117,60},{
          -104,60},{-104,-56},{-88,-56},{-88,-62}}, color={0,0,127}));
  connect(from_degC.y, mass_source.T_in) annotation (Line(points={{-117,26},{
          -102,26},{-102,46},{-96,46}}, color={0,0,127}));
  connect(from_degC.y, mass_source1.T_in) annotation (Line(points={{-117,26},{
          -106,26},{-106,-66},{-90,-66}}, color={0,0,127}));
  connect(pipe.port_a, relativePressure.port_a) annotation (Line(points={{18,40},
          {12,40},{12,42},{8,42},{8,74},{20,74}}, color={0,127,255}));
  connect(pipe.port_b, Tem_out.port_a) annotation (Line(points={{38,40},{42,40},
          {42,42},{54,42}}, color={0,127,255}));
  annotation (                                                       experiment(
      StopTime=17500,
      Interval=0.1,
      __Dymola_Algorithm="Dassl"));
end Test_pipe_model;
