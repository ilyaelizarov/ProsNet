within ProsNet.Under_Development.new_prosumer_models;
model heat_transfer_station

  replaceable package Medium_prim = ProsNet.Media.Water;
  replaceable package Medium_sec = ProsNet.Media.Water;

  extends ProsNet.Prosumers.BaseClasses.PrimarySideParameters;
  extends ProsNet.Prosumers.SecondarySides.BaseClasses.PumpsPairDynParam;
  extends ProsNet.Prosumers.SecondarySides.BaseClasses.ControlVolumeDynParam;

  Modelica.Blocks.Interfaces.RealVectorInput contr_vars_real[6]
annotation (Placement(transformation(extent={{-222,-20},{-182,20}})));

  Modelica.Blocks.Interfaces.RealVectorOutput states[8]
annotation (Placement(transformation(extent={{180,-20},{220,20}})));

  Real T_sec_in_set(unit="K", displayUnit="degC") "K"
annotation (Placement(transformation(extent={{-180,80},{-140,120}})));
  Real V_dot_sec_set(unit="l/min", displayUnit="l/min") "l/min"
annotation (Placement(transformation(extent={{-180,40},{-140,80}})));
  Real pi
annotation (Placement(transformation(extent={{-180,0},{-140,40}})));
  Real mu
annotation (Placement(transformation(extent={{-180,-40},{-140,0}})));
  Real u_set
annotation (Placement(transformation(extent={{-180,-80},{-140,-40}})));
  Real kappa_set
annotation (Placement(transformation(extent={{-180,-120},{-140,-80}})));

Real T_prim_hot(unit="K", displayUnit="degC")                                 "K"
                                               annotation (Placement(
    transformation(
    extent={{-20,-20},{20,20}},
    rotation=180,
    origin={160,140})));

  Real T_prim_cold(unit="K", displayUnit="degC") "K"
                                                annotation (Placement(
    transformation(
    extent={{-20,-20},{20,20}},
    rotation=180,
    origin={160,100})));

  Real T_sec_hot(
    unit="K",
    displayUnit="degC",
start=45 + 273.15)                                                         "K"
                                              annotation (Placement(
    transformation(
    extent={{-20,-20},{20,20}},
    rotation=180,
    origin={160,60})));

  Real T_sec_cold(
    unit="K",
    displayUnit="degC",
start=30 + 273.15)                                                           "K"
                                               annotation (Placement(
    transformation(
    extent={{-20,-20},{20,20}},
    rotation=180,
    origin={160,20})));

  Real V_dot_prim(unit="l/min", displayUnit="l/min") "l/min"
                                               annotation (Placement(
    transformation(
    extent={{-20,-20},{20,20}},
    rotation=180,
    origin={160,-20})));

  Real V_dot_sec(unit="l/min", displayUnit="l/min") "l/min"
                                              annotation (Placement(
    transformation(
    extent={{20,-20},{-20,20}},
    rotation=0,
    origin={160,-60})));

  Real Q_dot_is(unit="kW", displayUnit="kW")
"kW" annotation (Placement(transformation(
    extent={{20,-20},{-20,20}},
    rotation=0,
    origin={160,-100})));

  Real Delta_p_prim(unit="Pa", displayUnit="bar") annotation (
  Placement(transformation(
    extent={{-20,-20},{20,20}},
    rotation=180,
    origin={160,-140})));

  Real cp_prim;

  Fluid.HeatExchangers.LiquidToLiquid heat_exchanger(
redeclare package Medium2 = Medium_sec,
redeclare package Medium1 = Medium_prim,
m1_flow_nominal=m_flow_nominal_1,
m2_flow_nominal=m_flow_nominal_2,
show_T=true,
dp1_nominal=dp1_nominal,
dp2_nominal=dp2_nominal,
Q_flow_nominal=Q_flow_nominal,
T_a1_nominal=338.15,
T_a2_nominal=313.15)
annotation (Placement(transformation(extent={{30,12},{50,-8}})));

  Fluid.Pumps.FlowControlled_m_flow pump_sec_cons(
redeclare package Medium = Medium_sec,
final energyDynamics=energyDynamics_pumpsSec,
T_start=313.15,
final tau=tau_pumpsSec,
final m_flow_nominal=m_flow_nominal_2,
final use_inputFilter=use_inputFilter_pumpsSec,
final riseTime=riseTime_pumpsSec,
final dp_nominal=dp2_nominal,
final m_flow_start=m_flow_start_pumpsSec,
final y_start=y_start_pumpsSec) annotation (Placement(transformation(
    extent={{-10,10},{10,-10}},
    rotation=270,
    origin={60,70})));
  Fluid.Pumps.FlowControlled_m_flow pump_sec_prod(
redeclare package Medium = Medium_sec,
final energyDynamics=energyDynamics_pumpsSec,
T_start=313.15,
final tau=tau_pumpsSec,
final m_flow_nominal=m_flow_nominal_2,
final use_inputFilter=use_inputFilter_pumpsSec,
final riseTime=riseTime_pumpsSec,
final dp_nominal=dp2_nominal,
final m_flow_start=m_flow_start_pumpsSec,
final y_start=y_start_pumpsSec) annotation (Placement(transformation(
    extent={{-10,-10},{10,10}},
    rotation=90,
    origin={100,42})));
  Fluid.FixedResistances.CheckValve cheVa_sec_cons(
m_flow_nominal=m_flow_nominal_2,
redeclare final package Medium = Medium_sec,
final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
final Kv=Kv_cheVal,
final l=l_cheVal) annotation (Placement(transformation(
    extent={{-10,10},{10,-10}},
    rotation=270,
    origin={60,42})));
  Fluid.FixedResistances.CheckValve cheVal_sec_prod(
m_flow_nominal=m_flow_nominal_2,
redeclare final package Medium = Medium_sec,
final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
final Kv=Kv_cheVal,
final l=l_cheVal) annotation (Placement(transformation(
    extent={{10,-10},{-10,10}},
    rotation=270,
    origin={100,70})));
  Fluid.Valves.TwoWayEqualPercentage valve_prim_cons(
m_flow_nominal=m_flow_nominal_1,
redeclare final package Medium = Medium_prim,
final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
final Kv=Kv_conVal,
final use_inputFilter=use_inputFilter_conVal,
final riseTime=riseTime_conVal,
final init=init_conVal,
final y_start=y_start_conVal,
final l=l_conVal) annotation (Placement(transformation(
    extent={{10,10},{-10,-10}},
    rotation=-90,
    origin={60,-40})));
  Fluid.Pumps.SpeedControlled_y pump_prim_prod(
redeclare final package Medium = Medium_prim,
final energyDynamics=energyDynamics_feedPump,
T_start=313.15,
final tau=tau_feedPump,
final per=feedinPer,
final use_inputFilter=use_inputFilter_feedPump,
final riseTime=riseTime_feedPump,
final init=init_feedPump,
final y_start=y_start_feedPump) annotation (Placement(transformation(
    extent={{10,10},{-10,-10}},
    rotation=-90,
    origin={100,-70})));
  Fluid.FixedResistances.CheckValve cheVal_prim_prod(
m_flow_nominal=m_flow_nominal_1,
redeclare final package Medium = Medium_prim,
final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
final Kv=Kv_cheVal,
final l=l_cheVal) annotation (Placement(transformation(
    extent={{10,10},{-10,-10}},
    rotation=-90,
    origin={100,-40})));
  Fluid.FixedResistances.CheckValve cheVal_prim_cons(
m_flow_nominal=m_flow_nominal_1,
redeclare final package Medium = Medium_prim,
final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
final Kv=Kv_cheVal,
final l=l_cheVal) annotation (Placement(transformation(
    extent={{10,-10},{-10,10}},
    rotation=90,
    origin={62,-70})));
  Modelica.Fluid.Interfaces.FluidPort_b cold_prim(redeclare final package Medium =
        Medium_prim)
annotation (Placement(transformation(extent={{130,-190},{150,-170}})));
  Modelica.Fluid.Interfaces.FluidPort_a hot_prim(redeclare final package Medium =
        Medium_prim)
annotation (Placement(transformation(extent={{-150,-192},{-130,-172}})));
  heat_source_sink_ideal ideal_house(
energyDynamics_cv=Modelica.Fluid.Types.Dynamics.FixedInitial,
tau_cv=10,
T_start_cv=313.15)
annotation (Placement(transformation(extent={{18,120},{62,152}})));
  Conversion conversion
annotation (Placement(transformation(extent={{-100,-4},{-52,66}})));
  Modelica.Fluid.Sensors.MassFlowRate m_dot_sens_prim(
redeclare package Medium = Medium_prim, allowFlowReversal=true)
                                        annotation (Placement(
    transformation(
    extent={{10,-10},{-10,10}},
    rotation=90,
    origin={-20,-72})));
  Modelica.Fluid.Sensors.TemperatureTwoPort T_sens_prim_hot(
redeclare package Medium = Medium_prim, allowFlowReversal=true) annotation (
  Placement(transformation(
    extent={{-10,-10},{10,10}},
    rotation=90,
    origin={-20,-110})));
  Modelica.Fluid.Sensors.TemperatureTwoPort T_sens_prim_cold(
redeclare package Medium = Medium_prim, allowFlowReversal=true)
                                        annotation (
  Placement(transformation(
    extent={{-10,-10},{10,10}},
    rotation=-90,
    origin={80,-110})));
  Fluid.Sources.Boundary_pT bou(
    redeclare package Medium = Medium_sec,
T=313.15,
  nPorts=1)
annotation (Placement(transformation(extent={{128,118},{108,138}})));

  Fluid.Sensors.RelativePressure pressureDifference(redeclare package Medium =
        Medium_prim)
annotation (Placement(transformation(extent={{4,-164},{24,-144}})));

  Fluid.Pipes.InsulatedPipe_plug pipe_prim_hot(
T_amb=ambient_temperature,
R_ins=R_ins_transferpipe,
length=length_transfer_pipe_tot/2,
    diameter=d_transferpipe/2,
zeta=zeta_transferstation/2)  annotation (Placement(transformation(
    extent={{-10,-10},{10,10}},
    rotation=90,
    origin={-20,-138})));
  Fluid.Pipes.InsulatedPipe_plug pipe_prim_cold(
T_amb=ambient_temperature,
R_ins=R_ins_transferpipe,
length=length_transfer_pipe_tot/2,
    diameter=d_transferpipe/2,
zeta=zeta_transferstation/2)  annotation (Placement(transformation(
    extent={{-10,-10},{10,10}},
    rotation=-90,
    origin={80,-140})));
equation

  connect(
      contr_vars_real[1], conversion.T_sec_in_set);
  connect(
      contr_vars_real[2], conversion.V_dot_sec_set);
  connect(
      contr_vars_real[3], conversion.pi);
  connect(
      contr_vars_real[4], conversion.mu);
  connect(
      contr_vars_real[5], conversion.u_set);
  connect(
      contr_vars_real[6], conversion.kappa_set);

  T_sec_in_set = contr_vars_real[1];
  V_dot_sec_set = contr_vars_real[2];
  pi = contr_vars_real[3];
  mu = contr_vars_real[4];
  u_set = contr_vars_real[5];
  kappa_set = contr_vars_real[6];

  cp_prim = 4200;
  states[7] = m_dot_sens_prim.port_a.m_flow*cp_prim*(T_sens_prim_hot.T -
    T_sens_prim_cold.T)/1000;

  connect(
      heat_exchanger.port_b1, valve_prim_cons.port_b)
annotation (Line(points={{50,-4},{60,-4},{60,-30}}, color={0,127,255}));
  connect(
      cheVal_prim_prod.port_b,heat_exchanger. port_b1)
annotation (Line(points={{100,-30},{100,-4},{50,-4}},
                                                    color={0,127,255}));
  connect(
      cheVa_sec_cons.port_b,heat_exchanger. port_a2)
annotation (Line(points={{60,32},{60,8},{50,8}}, color={0,127,255}));
  connect(
      pump_sec_prod.port_a,heat_exchanger. port_a2)
annotation (Line(points={{100,32},{100,8},{50,8}},
                                                 color={0,127,255}));
  connect(
      pump_sec_cons.port_b, cheVa_sec_cons.port_a)
annotation (Line(points={{60,60},{60,52}}, color={0,127,255}));
  connect(
      cheVal_sec_prod.port_a, pump_sec_prod.port_b)
annotation (Line(points={{100,60},{100,52}},
                                           color={0,127,255}));
  connect(
      valve_prim_cons.port_a, cheVal_prim_cons.port_a)
annotation (Line(points={{60,-50},{60,-60},{62,-60}},
                                             color={0,127,255}));
  connect(
      cheVal_prim_prod.port_a, pump_prim_prod.port_b)
annotation (Line(points={{100,-50},{100,-60}},
                                             color={0,127,255}));
  connect(
      conversion.pump_contr, pump_prim_prod.y) annotation (Line(points={{-52,
          20.7059},{-38,20.7059},{-38,20},{-24,20},{-24,-24},{82,-24},{82,-70},
          {88,-70}},
                color={0,0,127}));
  connect(
      conversion.valve_contr, valve_prim_cons.y) annotation (Line(points={{-52,
          12.4706},{-42,12.4706},{-42,12},{-32,12},{-32,-40},{48,-40}},
                                                                   color={0,
      0,127}));
  connect(
      conversion.m_dot_cons, pump_sec_cons.m_flow_in) annotation (Line(
    points={{-52,37.1765},{42,37.1765},{42,70},{48,70}},
                                                      color={0,0,127}));
  connect(
      conversion.m_dot_prod, pump_sec_prod.m_flow_in) annotation (Line(
    points={{-52,45.4118},{-12,45.4118},{-12,46},{28,46},{28,24},{82,24},{82,42},
          {88,42}},    color={0,0,127}));


  connect(
      ideal_house.m_dot_sec_is, conversion.m_dot_sec_is);


  connect(
      m_dot_sens_prim.m_flow, conversion.m_dot_prim_is);
  connect(
      cheVal_prim_cons.port_b, T_sens_prim_cold.port_a) annotation (Line(
    points={{62,-80},{62,-90},{80,-90},{80,-100}}, color={0,127,255}));
  connect(
      pump_prim_prod.port_a, T_sens_prim_cold.port_a) annotation (Line(
    points={{100,-80},{100,-90},{80,-90},{80,-100}},
                                                   color={0,127,255}));


  connect(
      ideal_house.port_cold, cheVal_sec_prod.port_b) annotation (Line(
    points={{53.2,120},{54,120},{54,100},{80,100},{80,88},{100,88},{100,80}},
    color={0,127,255}));
  connect(
      ideal_house.port_cold, pump_sec_cons.port_a) annotation (Line(points={{53.2,
      120},{54,120},{54,100},{80,100},{80,88},{60,88},{60,80}},       color=
     {0,127,255}));
  connect(
      ideal_house.port_hot, heat_exchanger.port_b2) annotation (Line(points={{26.8,
      120},{32,120},{32,100},{0,100},{0,8},{30,8}},             color={0,127,
      255}));
  connect(
      bou.ports[1], cheVal_sec_prod.port_b)
annotation (Line(points={{108,128},{100,128},{100,80}},
                                                     color={0,127,255}));
  connect(
      hot_prim, pressureDifference.port_a) annotation (Line(points={{-140,-182},
      {-140,-154},{4,-154}},  color={0,127,255}));
  connect(
      pressureDifference.port_b, cold_prim) annotation (Line(points={{24,-154},
      {68,-154},{68,-166},{140,-166},{140,-180}},
                            color={0,127,255}));
  connect(
      heat_exchanger.port_a1, m_dot_sens_prim.port_a)
annotation (Line(points={{30,-4},{-20,-4},{-20,-62}},  color={0,127,255}));
  connect(
      m_dot_sens_prim.port_b, T_sens_prim_hot.port_b)
annotation (Line(points={{-20,-82},{-20,-100}}, color={0,127,255}));

  connect(
      T_sens_prim_hot.T, states[1]);
  connect(
      T_sens_prim_cold.T, states[2]);
  connect(
      ideal_house.T_sec_hot, states[3]);
  connect(
      ideal_house.T_sec_cold, states[4]);
  connect(
      conversion.V_dot_prim_is, states[5]);
  connect(
      conversion.V_dot_sec_is, states[6]);
  connect(
      pressureDifference.p_rel, states[8]);

  T_prim_hot = T_sens_prim_hot.T;
  T_prim_cold = T_sens_prim_cold.T;
  T_sec_hot = ideal_house.T_sec_hot;
  T_sec_cold = ideal_house.T_sec_cold;
  V_dot_prim = conversion.V_dot_prim_is;
  V_dot_sec = conversion.V_dot_sec_is;
  Q_dot_is = states[7];
  Delta_p_prim = pressureDifference.p_rel;

  connect(
      pipe_prim_hot.port_b, T_sens_prim_hot.port_a)
annotation (Line(points={{-20,-128},{-20,-120}}, color={0,127,255}));
  connect(
      pipe_prim_hot.port_a, hot_prim) annotation (Line(points={{-20,-148},{-20,-154},
      {-140,-154},{-140,-182}}, color={0,127,255}));
  connect(
      T_sens_prim_cold.port_b, pipe_prim_cold.port_a)
annotation (Line(points={{80,-120},{80,-130}}, color={0,127,255}));
  connect(
      pipe_prim_cold.port_b, cold_prim) annotation (Line(points={{80,-150},{80,-166},
      {140,-166},{140,-180}}, color={0,127,255}));
  annotation (Diagram(coordinateSystem(extent={{-200,-180},{200,180}})), Icon(
    coordinateSystem(extent={{-200,-180},{200,180}}), graphics={
    Line(
      points={{-140,-98},{-140,-170}},
      color={238,46,47},
      thickness=0.5),
    Line(
      points={{138,-100},{138,-168}},
      color={28,108,200},
      thickness=0.5),
    Rectangle(
      extent={{-38,30},{48,-48}},
      lineColor={0,0,0},
      lineThickness=1),
    Bitmap(extent={{-106,88},{108,176}}, fileName=
          "modelica://ProsNet/../../../../Downloads/noun-home-121812.svg"),
    Line(
      points={{32,86},{32,28}},
      color={28,108,200},
      thickness=0.5),
    Line(
      points={{-22,86},{-22,28}},
      color={238,46,47},
      thickness=0.5),
    Line(
      points={{138,-100},{38,-100}},
      color={28,108,200},
      thickness=0.5),
    Line(
      points={{-142,-100},{-24,-100}},
      color={238,46,47},
      thickness=0.5),
    Line(
      points={{-24,-52},{-24,-102}},
      color={238,46,47},
      thickness=0.5),
    Line(
      points={{36,-50},{36,-102}},
      color={28,108,200},
      thickness=0.5),
    Rectangle(extent={{-200,180},{200,-180}}, lineColor={0,0,0})}));
end heat_transfer_station;
