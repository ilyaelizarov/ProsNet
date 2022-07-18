within ProsNet.Under_Development.Tests;
model Test_PID_contr_prod_cons "Producer and Consumer with Controller"
  new_prosumer_models.heat_transfer_station producer(
    redeclare Fluid.Pumps.Data.Pumps.IMP.NMTDPlusER25slash90dash180 feedinPer,
    energyDynamics_feedPump=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    use_inputFilter_feedPump=true,
    init_feedPump=Modelica.Blocks.Types.Init.InitialOutput,
    use_inputFilter_conVal=true,
    init_conVal=Modelica.Blocks.Types.Init.InitialOutput,
    ambient_temperature=system.T_ambient,
    energyDynamics_pumpsSec=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    use_inputFilter_pumpsSec=true)
    annotation (Placement(transformation(extent={{-202,0},{-172,36}})));

  new_prosumer_models.heat_transfer_station consumer(
    redeclare Fluid.Pumps.Data.Pumps.IMP.NMTDPlusER25slash90dash180 feedinPer,
    energyDynamics_feedPump=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    use_inputFilter_feedPump=true,
    init_feedPump=Modelica.Blocks.Types.Init.InitialOutput,
    use_inputFilter_conVal=true,
    init_conVal=Modelica.Blocks.Types.Init.InitialOutput,
    ambient_temperature=system.T_ambient,
    energyDynamics_pumpsSec=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    use_inputFilter_pumpsSec=true,
    energyDynamics_cv=Modelica.Fluid.Types.Dynamics.SteadyState)
    annotation (Placement(transformation(extent={{-10,0},{-62,54}})));

  inner Modelica.Fluid.System system(T_ambient=285.15)
    annotation (Placement(transformation(extent={{70,-188},{90,-168}})));
  Fluid.Pipes.InsulatedPipe pipe_hot(allowFlowReversal=true, T_amb=system.T_ambient,
    length=180,
    zeta=1000)
    annotation (Placement(transformation(extent={{-126,-94},{-106,-74}})));
  Fluid.Pipes.InsulatedPipe pipe_cold(
    allowFlowReversal=true,
    T_amb=system.T_ambient,
    length=180,
    zeta=1000,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
    annotation (Placement(transformation(extent={{-106,-58},{-126,-38}})));
  Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare final package Medium =
        Media.Water)
    annotation (Placement(transformation(extent={{70,-94},{50,-74}})));
  Controller_PID_based.PID_Q_T_weighted PID_Q_T_weighted_2(
    V_dot_sec_max=10,
    alpha_prim_prod=1,
    alpha_sec_prod=1,
    alpha_prim_cons=1,
    alpha_sec_cons=1)
    annotation (Placement(transformation(extent={{-268,-6},{-226,42}})));
  Controller_PID_based.auxiliary.TimeTable_noInterp T_sec_in_array(table=[0,
        273.15 + 45; 900,273.15 + 45; 1800,273.15 + 45; 2700,273.15 + 45; 3600,
        273.15 + 45; 4500,273.15 + 45; 5400,273.15 + 45; 6300,273.15 + 45; 7200,
        273.15 + 45; 8100,273.15 + 45; 9000,273.15 + 30; 9900,273.15 + 30;
        10800,273.15 + 30; 11700,273.15 + 30; 12600,273.15 + 45; 13500,273.15
         + 45; 14400,273.15 + 45; 15300,273.15 + 30; 16200,273.15 + 30; 17100,
        273.15 + 30; 18000,273.15 + 30])
    annotation (Placement(transformation(extent={{-400,32},{-380,52}})));
  Controller_PID_based.PID_Q_T_weighted PID_Q_T_weighted_1(
    V_dot_sec_max=10,
    alpha_prim_prod=1,
    alpha_sec_prod=1,
    alpha_prim_cons=1,
    alpha_sec_cons=1)
    annotation (Placement(transformation(extent={{64,6},{22,54}})));
  Controller_PID_based.auxiliary.TimeTable_noInterp Q_management_array(table=[0,
        0; 900,3; 1800,3; 2700,3; 3600,0; 4500,0; 5400,0; 6300,3; 7200,5; 8100,
        3; 9000,0; 9900,-3; 10800,-5; 11700,-3; 12600,0; 13500,5; 14400,0;
        15300,0; 16200,-5; 17100,0; 18000,0])
    annotation (Placement(transformation(extent={{-314,8},{-294,28}})));
  Controller_PID_based.auxiliary.TimeTable_noInterp Q_management_array2(table=[0,
        0; 900,-3; 1800,-3; 2700,-3; 3600,0; 4500,0; 5400,0; 6300,-3; 7200,-5;
        8100,-3; 9000,0; 9900,3; 10800,5; 11700,3; 12600,0; 13500,-5; 14400,0;
        15300,0; 16200,5; 17100,0; 18000,0])
    annotation (Placement(transformation(extent={{126,-6},{106,14}})));
  inner Modelica.Blocks.Noise.GlobalSeed globalSeed(enableNoise=false,
      fixedSeed=4345)
    annotation (Placement(transformation(extent={{-126,72},{-106,92}})));
  Modelica.Blocks.Noise.NormalNoise normalNoise(
    samplePeriod=30,
    mu=0,
    sigma=3)
    annotation (Placement(transformation(extent={{-400,70},{-380,90}})));
  Modelica.Blocks.Math.Add add
    annotation (Placement(transformation(extent={{-356,54},{-336,74}})));
  Modelica.Blocks.Noise.NormalNoise normalNoise1(
    samplePeriod=30,
    mu=0,
    sigma=3) annotation (Placement(transformation(extent={{202,62},{182,82}})));
  Modelica.Blocks.Math.Add add1
    annotation (Placement(transformation(extent={{156,40},{136,60}})));
  Controller_PID_based.auxiliary.TimeTable_noInterp T_sec_in_array1(table=[0,
        273.15 + 30; 900,273.15 + 30; 1800,273.15 + 30; 2700,273.15 + 30; 3600,
        273.15 + 30; 4500,273.15 + 30; 5400,273.15 + 30; 6300,273.15 + 30; 7200,
        273.15 + 30; 8100,273.15 + 30; 9000,273.15 + 45; 9900,273.15 + 45;
        10800,273.15 + 45; 11700,273.15 + 45; 12600,273.15 + 30; 13500,273.15
         + 30; 14400,273.15 + 30; 15300,273.15 + 45; 16200,273.15 + 45; 17100,
        273.15 + 45; 18000,273.15 + 45])
    annotation (Placement(transformation(extent={{190,-6},{170,14}})));
equation
  connect(pipe_hot.port_a, producer.hot_prim) annotation (Line(points={{-126,
          -84},{-188,-84},{-188,-0.2}}, color={0,127,255}));
  connect(pipe_hot.port_b, consumer.hot_prim) annotation (Line(points={{-106,
          -84},{-34.2667,-84},{-34.2667,-0.3}},
                                      color={0,127,255}));
  connect(pipe_cold.port_a, consumer.cold_prim) annotation (Line(points={{-106,
          -48},{-50,-48},{-50,-4},{-51.6,-4},{-51.6,-0.3}},
                                           color={0,127,255}));
  connect(producer.cold_prim, pipe_cold.port_b) annotation (Line(points={{-178,
          -0.2},{-178,-48},{-126,-48}},      color={0,127,255}));
  connect(bou.ports[1], pipe_hot.port_b)
    annotation (Line(points={{50,-84},{-106,-84}}, color={0,127,255}));
  connect(PID_Q_T_weighted_2.T_sec_set, producer.T_sec_in_set) annotation (
      Line(points={{-226,42},{-226,52},{-210,52},{-210,32},{-202,32}}, color=
          {0,0,127}));
  connect(PID_Q_T_weighted_2.V_dot_sec_set, producer.V_dot_sec_set)
    annotation (Line(points={{-226,32.4},{-212,32.4},{-212,28},{-202,28}},
        color={0,0,127}));
  connect(PID_Q_T_weighted_2.pi_set, producer.pi)
    annotation (Line(points={{-226,22.8},{-202,22}}, color={255,127,0}));
  connect(PID_Q_T_weighted_2.mu_set, producer.mu) annotation (Line(points={{-226,
          13.2},{-210,13.2},{-210,18},{-202,18}}, color={255,127,0}));
  connect(producer.u_set, PID_Q_T_weighted_2.u_set) annotation (Line(points={
          {-202,14},{-208,14},{-208,3.6},{-226,3.6}}, color={0,0,127}));
  connect(producer.kappa_set, PID_Q_T_weighted_2.kappa_set) annotation (Line(
        points={{-202,10},{-212,10},{-212,-16},{-226,-16},{-226,-6}}, color={
          0,0,127}));
  connect(producer.T_sec_hot, PID_Q_T_weighted_2.T_sec_hot) annotation (Line(
        points={{-190,36},{-190,54},{-252,54},{-252,46},{-253.3,46},{-253.3,
          42}}, color={0,0,127}));
  connect(producer.T_sec_cold, PID_Q_T_weighted_2.T_sec_cold) annotation (
      Line(points={{-174,36},{-174,44},{-218,44},{-218,48},{-240.7,48},{-240.7,
          42}}, color={0,0,127}));
  connect(producer.V_dot_sec, PID_Q_T_weighted_2.V_dot_sec) annotation (Line(
        points={{-182,36},{-182,58},{-278,58},{-278,32.4},{-268,32.4}}, color=
         {0,0,127}));
  connect(producer.T_prim_hot, PID_Q_T_weighted_2.T_prim_hot) annotation (
      Line(points={{-190,0},{-190,-18},{-252,-18},{-252,-10},{-253.3,-10},{-253.3,
          -6}}, color={0,0,127}));
  connect(producer.T_prim_cold, PID_Q_T_weighted_2.T_prim_cold) annotation (
      Line(points={{-174,0},{-174,-6},{-218,-6},{-218,-12},{-240.7,-12},{-240.7,
          -6}}, color={0,0,127}));
  connect(PID_Q_T_weighted_2.V_dot_prim, producer.V_dot_prim) annotation (
      Line(points={{-268,3.6},{-278,3.6},{-278,-22},{-182,-22},{-182,0}},
        color={0,0,127}));
  connect(producer.Q_dot_trnsf_is, PID_Q_T_weighted_2.Qdot_is) annotation (
      Line(points={{-202,4},{-214,4},{-214,-24},{-280,-24},{-280,13.2},{-268,
          13.2}}, color={0,0,127}));
  connect(PID_Q_T_weighted_1.T_sec_set, consumer.T_sec_in_set) annotation (
      Line(points={{22,54},{22,64},{0,64},{0,48},{-10,48}}, color={0,0,127}));
  connect(PID_Q_T_weighted_1.V_dot_sec_set, consumer.V_dot_sec_set)
    annotation (Line(points={{22,44.4},{0,44.4},{0,42},{-10,42}}, color={0,
          0,127}));
  connect(PID_Q_T_weighted_1.pi_set, consumer.pi)
    annotation (Line(points={{22,34.8},{-10,33}}, color={255,127,0}));
  connect(PID_Q_T_weighted_1.mu_set, consumer.mu) annotation (Line(points={
          {22,25.2},{0,25.2},{0,27},{-10,27}}, color={255,127,0}));
  connect(PID_Q_T_weighted_1.u_set, consumer.u_set) annotation (Line(points=
         {{22,15.6},{0,15.6},{0,21},{-10,21}}, color={0,0,127}));
  connect(PID_Q_T_weighted_1.kappa_set, consumer.kappa_set) annotation (
      Line(points={{22,6},{14,6},{14,-6},{-2,-6},{-2,14},{-10,14},{-10,15}},
        color={0,0,127}));
  connect(consumer.T_sec_hot, PID_Q_T_weighted_1.T_sec_hot) annotation (
      Line(points={{-30.8,54},{-30.8,68},{48,68},{48,58},{49.3,58},{49.3,54}},
        color={0,0,127}));
  connect(consumer.T_sec_cold, PID_Q_T_weighted_1.T_sec_cold) annotation (
      Line(points={{-58.5333,54},{-56,54},{-56,66},{36.7,66},{36.7,54}},
        color={0,0,127}));
  connect(consumer.V_dot_sec, PID_Q_T_weighted_1.V_dot_sec) annotation (
      Line(points={{-44.6667,54},{-44.6667,72},{74,72},{74,44.4},{64,44.4}},
        color={0,0,127}));
  connect(consumer.T_prim_hot, PID_Q_T_weighted_1.T_prim_hot) annotation (
      Line(points={{-30.8,0},{-30.8,-10},{48,-10},{48,2},{49.3,2},{49.3,6}},
        color={0,0,127}));
  connect(consumer.T_prim_cold, PID_Q_T_weighted_1.T_prim_cold) annotation (
     Line(points={{-58.5333,0},{-60,0},{-60,-16},{36.7,-16},{36.7,6}},
        color={0,0,127}));
  connect(consumer.V_dot_prim, PID_Q_T_weighted_1.V_dot_prim) annotation (
      Line(points={{-44.6667,0},{-46,0},{-46,-14},{74,-14},{74,15.6},{64,15.6}},
                  color={0,0,127}));
  connect(consumer.Q_dot_trnsf_is, PID_Q_T_weighted_1.Qdot_is) annotation (
      Line(points={{-10,6},{-4,6},{-4,-18},{76,-18},{76,25.2},{64,25.2}},
        color={0,0,127}));
  connect(Q_management_array.y, PID_Q_T_weighted_2.Qdot_set) annotation (Line(
        points={{-293,18},{-278,18},{-278,22.8},{-268,22.8}}, color={0,0,127}));
  connect(Q_management_array2.y, PID_Q_T_weighted_1.Qdot_set) annotation (Line(
        points={{105,4},{84,4},{84,34.8},{64,34.8}}, color={0,0,127}));
  connect(normalNoise.y, add.u1) annotation (Line(points={{-379,80},{-366,80},{
          -366,70},{-358,70}}, color={0,0,127}));
  connect(T_sec_in_array.y, add.u2) annotation (Line(points={{-379,42},{-366,42},
          {-366,58},{-358,58}}, color={0,0,127}));
  connect(add.y, PID_Q_T_weighted_2.T_sec_sim) annotation (Line(points={{-335,
          64},{-318,64},{-318,42},{-268,42}}, color={0,0,127}));
  connect(normalNoise1.y, add1.u1) annotation (Line(points={{181,72},{168,72},{
          168,56},{158,56}}, color={0,0,127}));
  connect(add1.y, PID_Q_T_weighted_1.T_sec_sim) annotation (Line(points={{135,
          50},{86,50},{86,56},{64,56},{64,54}}, color={0,0,127}));
  connect(T_sec_in_array1.y, add1.u2) annotation (Line(points={{169,4},{160,4},
          {160,34},{166,34},{166,44},{158,44}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-320,
            -200},{140,100}})), Diagram(coordinateSystem(preserveAspectRatio=
            false, extent={{-320,-200},{140,100}}), graphics={
        Text(
          extent={{-214,82},{-146,68}},
          textColor={238,46,47},
          textString="Producer"),
        Text(
          extent={{-62,84},{6,70}},
          textColor={28,108,200},
          textString="Consumer"),
        Text(
          extent={{-134,-30},{-98,-38}},
          textColor={28,108,200},
          textString="cold"),
        Text(
          extent={{-134,-106},{-98,-114}},
          textColor={238,46,47},
          textString="hot")}),
    experiment(
      StopTime=18900,
      Interval=30,
      __Dymola_Algorithm="Dassl"));
end Test_PID_contr_prod_cons;
