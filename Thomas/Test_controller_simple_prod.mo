within ;
model Test_controller_simple_prod

  inner Modelica.Fluid.System system
    annotation (Placement(transformation(extent={{-90,68},{-70,88}})));
  ProsNet.Prosumers.ProsumerIdeal pros(
    use_mu_set_in=true,
    use_pi_set_in=true,
    use_kappa_set_in=true,
    use_u_set_in=true,
    use_inputFilter_feedPump=true,
    riseTime_feedPump=5,
    use_inputFilter_conVal=true,
    riseTime_conVal=5,
    T_sec=353.15)
    annotation (Placement(transformation(extent={{-14,30},{6,50}})));
  controller_simple controller_simple1
    annotation (Placement(transformation(extent={{-68,28},{-48,48}})));
  Modelica.Blocks.Sources.IntegerExpression integerExpression(y=1)
    annotation (Placement(transformation(extent={{-106,28},{-86,48}})));
  Modelica.Fluid.Pipes.DynamicPipe pipe1(
    redeclare package Medium = ProsNet.Media.Water,
    nParallel=1,
    length=20,
    isCircular=true,
    diameter(displayUnit="mm") = 0.022) annotation (Placement(transformation(
        extent={{-10,-11},{10,11}},
        rotation=90,
        origin={-44,-29})));
  Modelica.Fluid.Vessels.OpenTank tank(
    height=1,
    crossArea=1,
    redeclare package Medium = ProsNet.Media.Water,
    T_start=293.15,
    use_portsData=false,
    nPorts=2) annotation (Placement(transformation(extent={{-2,-58},{38,-18}})));
equation
  connect(integerExpression.y, controller_simple1.prosumer_mode)
    annotation (Line(points={{-85,38},{-68,38}}, color={255,127,0}));
  connect(controller_simple1.pi, pros.pi_set) annotation (Line(points={{-48,44},
          {-22,44},{-22,52},{-16.6,52}}, color={255,127,0}));
  connect(controller_simple1.mu, pros.mu_set) annotation (Line(points={{-48,42},
          {-26,42},{-26,58},{-12,58},{-12,52}}, color={255,127,0}));
  connect(controller_simple1.u, pros.u_set) annotation (Line(points={{-48,36},{
          -36,36},{-36,60},{4,60},{4,56},{5,56},{5,52}}, color={0,0,127}));
  connect(controller_simple1.kappa, pros.kappa_set) annotation (Line(points={{
          -48,34},{-38,34},{-38,62},{8,62},{8,56},{9.6,56},{9.6,52}}, color={0,
          0,127}));
  connect(pros.T_sec_cold, controller_simple1.T_sec_cold) annotation (Line(
        points={{-15,34},{-38,34},{-38,46},{-40,46},{-40,54},{-56,54},{-56,48}},
        color={0,0,127}));
  connect(pros.T_sec_hot, controller_simple1.T_sec_hot) annotation (Line(points=
         {{7,34},{16,34},{16,66},{-62,66},{-62,48}}, color={0,0,127}));
  connect(pros.T_prim_cold, controller_simple1.T_prim_cold) annotation (Line(
        points={{-15,46},{-40,46},{-40,22},{-56,22},{-56,28}}, color={0,0,127}));
  connect(pros.T_prim_hot, controller_simple1.T_prim_hot) annotation (Line(
        points={{7,46},{12,46},{12,22},{-38,22},{-38,20},{-62,20},{-62,28}},
        color={0,0,127}));
  connect(pros.port_b, tank.ports[1]) annotation (Line(points={{6,40},{42,40},{
          42,-66},{14,-66},{14,-58}}, color={0,127,255}));
  connect(tank.ports[2], pipe1.port_a) annotation (Line(points={{22,-58},{14,
          -58},{14,-64},{-44,-64},{-44,-39}}, color={0,127,255}));
  connect(pipe1.port_b, pros.port_a)
    annotation (Line(points={{-44,-19},{-44,40},{-14,40}}, color={0,127,255}));
  annotation (uses(
      Modelica(version="4.0.0"),
      ProsNet(version="1"),
      controller_simple(version="1")), experiment(
      StopTime=100,
      Interval=0.1,
      __Dymola_Algorithm="Dassl"));
end Test_controller_simple_prod;
