within ProsNet;
package BidirectionalSubstation
  package Substation_idealProsumer
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
      Modelica.Fluid.Interfaces.FluidPort_b cold_prim(redeclare final package
          Medium =
            Medium_prim)
    annotation (Placement(transformation(extent={{130,-190},{150,-170}})));
      Modelica.Fluid.Interfaces.FluidPort_a hot_prim(redeclare final package Medium =
            Medium_prim)
    annotation (Placement(transformation(extent={{-150,-192},{-130,-172}})));
      Under_Development.new_prosumer_models.Conversion conversion
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
      Prosumers.heat_source_sink_ideal heat_source_sink_ideal
        annotation (Placement(transformation(extent={{14,126},{64,162}})));
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
              20.7059},{-38,20.7059},{-38,20},{-24,20},{-24,-24},{82,-24},{82,
              -70},{88,-70}},
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
        points={{-52,45.4118},{-12,45.4118},{-12,46},{28,46},{28,24},{82,24},{
              82,42},{88,42}},
                           color={0,0,127}));

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
      connect(heat_source_sink_ideal.port_cold, pump_sec_cons.port_a) annotation (
          Line(points={{54,126},{54,86},{60,86},{60,80}}, color={0,127,255}));
      connect(heat_exchanger.port_b2, heat_source_sink_ideal.port_hot)
        annotation (Line(points={{30,8},{24,8},{24,126}}, color={0,127,255}));
      connect(conversion.T_sec_in, heat_source_sink_ideal.T_set) annotation (Line(
            points={{-66.4,66},{-66.4,172},{39,172},{39,162}}, color={0,0,127}));
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

    model Conversion

    package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater;

      Modelica.Blocks.Sources.Constant factor1(k=(1/60000)*Medium.d_const)
        annotation (Placement(transformation(extent={{-52,40},{-32,60}})));
      Modelica.Blocks.Math.Product volume2mass_flow
        annotation (Placement(transformation(extent={{-10,22},{10,42}})));
      Controls.SecondaryFlowControl secFlowCon
        annotation (Placement(transformation(extent={{30,4},{50,24}})));
      Controls.PrimaryFlowControl priFlowCon
        annotation (Placement(transformation(extent={{-8,-26},{12,-46}})));
      Controls.Linearizer         lin(redeclare final
          Controls.Data.Linearizer.EqualPercentage cha)
       annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-30,-70})));
      Modelica.Blocks.Interfaces.RealInput T_sec_in_set "Kelvin"
        annotation (Placement(transformation(extent={{-120,120},{-80,160}})));
      Modelica.Blocks.Interfaces.RealInput V_dot_sec_set "l/min"
        annotation (Placement(transformation(extent={{-120,80},{-80,120}})));
      Modelica.Blocks.Interfaces.RealInput pi "{0;1}"
        annotation (Placement(transformation(extent={{-120,-20},{-80,20}})));
      Modelica.Blocks.Interfaces.RealInput mu "{-1;0;1}"
        annotation (Placement(transformation(extent={{-120,-62},{-80,-22}})));
      Modelica.Blocks.Interfaces.RealInput kappa_set "[0;1]"
        annotation (Placement(transformation(extent={{-120,-140},{-80,-100}})));
      Modelica.Blocks.Interfaces.RealInput u_set "[0;1]"
        annotation (Placement(transformation(extent={{-120,-100},{-80,-60}})));
      Modelica.Blocks.Interfaces.RealInput m_dot_sec_is "kg/s"
        annotation (Placement(transformation(extent={{120,100},{80,140}})));
      Modelica.Blocks.Interfaces.RealOutput m_dot_prod "kg/s"
        annotation (Placement(transformation(extent={{90,70},{110,90}})));
      Modelica.Blocks.Interfaces.RealOutput m_dot_cons "kg/s"
        annotation (Placement(transformation(extent={{90,30},{110,50}})));
      Modelica.Blocks.Interfaces.RealOutput pump_contr "[0;1]"
        annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
      Modelica.Blocks.Interfaces.RealOutput valve_contr "[0;1]"
        annotation (Placement(transformation(extent={{90,-90},{110,-70}})));
      Modelica.Blocks.Interfaces.RealOutput T_sec_in annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={40,180})));
      Modelica.Blocks.Interfaces.RealOutput V_dot_sec_is annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={0,180})));
      Modelica.Blocks.Math.Product mass2volume_flow
        annotation (Placement(transformation(extent={{36,50},{16,70}})));
      Modelica.Blocks.Sources.Constant factor2(k=(60000)*(1/Medium.d_const))
        annotation (Placement(transformation(extent={{76,34},{56,54}})));
      Modelica.Blocks.Interfaces.RealInput m_dot_prim_is "kg/s"
        annotation (Placement(transformation(extent={{120,-140},{80,-100}})));
      Modelica.Blocks.Interfaces.RealOutput V_dot_prim_is annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={0,-160})));
      Modelica.Blocks.Math.Product mass2volume_flow1
        annotation (Placement(transformation(extent={{48,-132},{28,-112}})));
      Modelica.Blocks.Sources.Constant factor3(k=(60000)*(1/Medium.d_const))
        annotation (Placement(transformation(extent={{82,-108},{62,-88}})));
      Modelica.Blocks.Math.RealToInteger realToInteger
        annotation (Placement(transformation(extent={{-72,-10},{-52,10}})));
      Modelica.Blocks.Math.RealToInteger realToInteger1
        annotation (Placement(transformation(extent={{-72,-52},{-52,-32}})));
    equation
      connect(factor1.y, volume2mass_flow.u1) annotation (Line(points={{-31,50},{-18,
              50},{-18,38},{-12,38}}, color={0,0,127}));
      connect(T_sec_in_set, T_sec_in)
        annotation (Line(points={{-100,140},{40,140},{40,180}}, color={0,0,127}));
      connect(volume2mass_flow.u2, V_dot_sec_set) annotation (Line(points={{-12,26},
              {-74,26},{-74,100},{-100,100}}, color={0,0,127}));
      connect(factor2.y, mass2volume_flow.u2) annotation (Line(points={{55,44},{44,44},
              {44,54},{38,54}}, color={0,0,127}));
      connect(m_dot_sec_is, mass2volume_flow.u1) annotation (Line(points={{100,120},
              {48,120},{48,66},{38,66}}, color={0,0,127}));
      connect(mass2volume_flow.y, V_dot_sec_is)
        annotation (Line(points={{15,60},{0,60},{0,180}}, color={0,0,127}));
      connect(volume2mass_flow.y, secFlowCon.m_flow_set)
        annotation (Line(points={{11,32},{40,32},{40,26}}, color={0,0,127}));
      connect(secFlowCon.m_flow_production, m_dot_prod) annotation (Line(points={{51,19.1},
              {51,32},{84,32},{84,64},{74,64},{74,80},{100,80}},       color={0,0,127}));
      connect(secFlowCon.m_flow_consumption, m_dot_cons) annotation (Line(points={{51,8.9},
              {62,8.9},{62,28},{82,28},{82,40},{100,40}},      color={0,0,127}));
      connect(priFlowCon.pump_y, pump_contr) annotation (Line(points={{13,-30.9},{84,
              -30.9},{84,-40},{100,-40}}, color={0,0,127}));
      connect(priFlowCon.valve_op, valve_contr) annotation (Line(points={{13,-41.1},
              {82,-41.1},{82,-80},{100,-80}}, color={0,0,127}));
      connect(lin.op, priFlowCon.valve_op_set)
        annotation (Line(points={{-19,-70},{6,-70},{6,-48}}, color={0,0,127}));
      connect(kappa_set, lin.kappa) annotation (Line(points={{-100,-120},{-52,-120},
              {-52,-70},{-42,-70}}, color={0,0,127}));
      connect(u_set, priFlowCon.pump_y_set) annotation (Line(points={{-100,-80},{-54,
              -80},{-54,-54},{-10,-54},{-10,-56},{-2,-56},{-2,-48}}, color={0,0,127}));
      connect(factor3.y, mass2volume_flow1.u1) annotation (Line(points={{61,-98},
              {60,-98},{60,-116},{50,-116}},
                                         color={0,0,127}));
      connect(m_dot_prim_is, mass2volume_flow1.u2) annotation (Line(points={{100,
              -120},{58,-120},{58,-128},{50,-128}},
                                              color={0,0,127}));
      connect(mass2volume_flow1.y, V_dot_prim_is)
        annotation (Line(points={{27,-122},{0,-122},{0,-160}}, color={0,0,127}));
      connect(realToInteger.y, secFlowCon.pi)
        annotation (Line(points={{-51,0},{20,0},{20,8},{28,8}}, color={255,127,0}));
      connect(realToInteger.y, priFlowCon.pi)
        annotation (Line(points={{-51,0},{-24,0},{-24,-30},{-10,-30}}, color={255,127,0}));
      connect(realToInteger.u, pi)
        annotation (Line(points={{-74,0},{-100,0}}, color={0,0,127}));
      connect(mu, realToInteger1.u)
        annotation (Line(points={{-100,-42},{-74,-42}}, color={0,0,127}));
      connect(realToInteger1.y, priFlowCon.mu)
        annotation (Line(points={{-51,-42},{-10,-42}}, color={255,127,0}));
      connect(realToInteger1.y, secFlowCon.mu) annotation (Line(points={{-51,-42},{-32,-42},
              {-32,14},{2,14},{2,20},{28,20}}, color={255,127,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-160},
                {100,180}}), graphics={Rectangle(
              extent={{-100,180},{100,-160}},
              lineColor={28,108,200},
              fillColor={171,171,171},
              fillPattern=FillPattern.Solid), Text(
              extent={{-137,35},{137,-35}},
              textColor={0,0,0},
              origin={-1,1},
              rotation=90,
              textString="Conversion",
              textStyle={TextStyle.Bold})}),
                              Diagram(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-160},{100,180}})));
    end Conversion;

    model Test_heat_source_sink_ideal
      Modelica.Fluid.Sources.MassFlowSource_T boundary(
        use_m_flow_in=true,
        use_T_in=true,
        redeclare package Medium =
            Media.Water,
        nPorts=1) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-20,-70})));
      Modelica.Fluid.Vessels.OpenTank tank(
        height=5,
        crossArea=10,
        use_portsData=false,
        redeclare package Medium =
            Media.Water,
        nPorts=1) annotation (Placement(transformation(extent={{30,-76},{70,-34}})));
      Modelica.Blocks.Sources.RealExpression temperature_sec(y=273.15 + 70)
        "Kelvin"
        annotation (Placement(transformation(extent={{-68,68},{-48,88}})));
      Modelica.Blocks.Sources.RealExpression temperature_in(y=273.15 + 30)
        "Kelvin"
        annotation (Placement(transformation(extent={{-96,-88},{-76,-68}})));
      Modelica.Blocks.Sources.RealExpression m_flow(y=5/60) "kg/s"
        annotation (Placement(transformation(extent={{-96,-70},{-76,-50}})));
      inner Modelica.Fluid.System system
        annotation (Placement(transformation(extent={{56,62},{76,82}})));
      Prosumers.heat_source_sink_ideal heat_source_sink_ideal
        annotation (Placement(transformation(extent={{-30,-4},{22,42}})));
    equation
      connect(boundary.m_flow_in, m_flow.y) annotation (Line(points={{-28,-80},{
              -44,-80},{-44,-60},{-75,-60}}, color={0,0,127}));
      connect(boundary.T_in, temperature_in.y) annotation (Line(points={{-24,-82},
              {-24,-88},{-70,-88},{-70,-78},{-75,-78}}, color={0,0,127}));
      connect(temperature_sec.y, heat_source_sink_ideal.T_set)
        annotation (Line(points={{-47,78},{-4,78},{-4,42}}, color={0,0,127}));
      connect(heat_source_sink_ideal.port_hot, tank.ports[1]) annotation (Line(
            points={{-19.6,-4},{-19.6,-30},{24,-30},{24,-84},{50,-84},{50,-76}},
            color={0,127,255}));
      connect(heat_source_sink_ideal.port_cold, boundary.ports[1]) annotation (
          Line(points={{11.6,-4},{11.6,-54},{-20,-54},{-20,-60}}, color={0,127,
              255}));
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
            coordinateSystem(preserveAspectRatio=false)));
    end Test_heat_source_sink_ideal;

    model Test_heat_transfer_station_production
      Modelica.Blocks.Math.Add add annotation (Placement(transformation(
            extent={{-5,-5},{5,5}},
            rotation=-90,
            origin={-1,41})));
      Modelica.Blocks.Sources.RealExpression realExpression(y=273.15) annotation (
          Placement(transformation(
            extent={{-5,-6},{5,6}},
            rotation=270,
            origin={22,55})));
      ProsNet.Under_Development.Controller_PID_based.auxiliary.TimeTable_noInterp
                                                        power_set1(table=[0,4;
            3600,-4; 7200,10; 10800,-10; 14400,-6; 18000,6; 21600,10; 25200,10])
                                                                   annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={-36,76})));
      ProsNet.Under_Development.Controller_PID_based.auxiliary.TimeTable_noInterp
                                                        temp_sec_in1(table=[0,
            55; 3600,30; 7200,55; 10800,30; 14400,30; 18000,55; 21600,55; 25200,
            55])                                                     annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={6,76})));
      ProsNet.Under_Development.Controller_PID_based.PID_Q_T_weighted_crossover
                                                      Ctrl1(
        alpha_prim_prod=1,
        alpha_sec_prod=0,
        alpha_prim_cons=0,
        alpha_sec_cons=1)
        annotation (Placement(transformation(extent={{-28,-16},{-4,18}})));
      Modelica.Blocks.Sources.Ramp ramp(
        height=1,
        duration=600,
        offset=0,
        startTime=200)
        annotation (Placement(transformation(extent={{-64,-186},{-44,-166}})));
      Modelica.Fluid.Vessels.ClosedVolume volume(
        T_start=318.15,
        use_portsData=false,
        V=1,
        nPorts=2,
        redeclare final package Medium = ProsNet.Media.Water)
        annotation (Placement(transformation(extent={{30,-178},{50,-198}})));
      ProsNet.Fluid.Valves.TwoWayEqualPercentage
                                         valve_for_test(
        m_flow_nominal=30.074213*0.001/60,
        redeclare final package Medium = ProsNet.Media.Water,
        final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
        final Kv=2.5,
        final use_inputFilter=true,
        final riseTime=5,
        final init=Modelica.Blocks.Types.Init.InitialOutput,
        final y_start=0,
        final l=2e-3) annotation (Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=-90,
            origin={64,-158})));
      ProsNet.Fluid.Sources.Boundary_pT
                                bou(nPorts=1, redeclare final package Medium =
            ProsNet.Media.Water)
        annotation (Placement(transformation(extent={{134,-152},{114,-132}})));
      heat_transfer_station heat_transfer_station1(redeclare
          Fluid.Pumps.Data.Pumps.IMP.NMTSmart25_120to180 feedinPer,
          R_ins_transferpipe=100000)
        annotation (Placement(transformation(extent={{-36,-64},{4,-28}})));
    equation
      connect(add.u1,realExpression. y) annotation (Line(points={{2,47},{2,49.5},
              {22,49.5}},                color={0,0,127}));
      connect(temp_sec_in1.y,add. u2) annotation (Line(points={{6,65},{6,47},{
              -4,47}},         color={0,0,127}));
      connect(power_set1.y,Ctrl1. Q_dot_set) annotation (Line(points={{-36,65},
              {-36,28},{-22,28},{-22,18.8}},
                                    color={0,0,127}));
      connect(add.y,Ctrl1. T_sec_in_is) annotation (Line(points={{-1,35.5},{-2,
              35.5},{-2,28},{-10,28},{-10,19}},
                             color={0,0,127}));
      connect(valve_for_test.port_a,volume. ports[1])
        annotation (Line(points={{64,-168},{64,-170},{39,-170},{39,-178}},
                                                              color={0,127,255}));
      connect(ramp.y,valve_for_test. y) annotation (Line(points={{-43,-176},{26,
              -176},{26,-158},{52,-158}},
                              color={0,0,127}));
      connect(bou.ports[1],valve_for_test. port_b)
        annotation (Line(points={{114,-142},{64,-142},{64,-148}},
                                                          color={0,127,255}));
      connect(Ctrl1.contr_vars_real[1], heat_transfer_station1.contr_vars_real[
        1]) annotation (Line(points={{-4,-0.833333},{12,-0.833333},{12,-46.8333},
              {-36.2,-46.8333}}, color={0,0,127}));
      connect(Ctrl1.contr_vars_real[2], heat_transfer_station1.contr_vars_real[
        2]) annotation (Line(points={{-4,-0.5},{12,-0.5},{12,-46.5},{-36.2,
              -46.5}}, color={0,0,127}));
      connect(Ctrl1.contr_vars_real[3], heat_transfer_station1.contr_vars_real[
        3]) annotation (Line(points={{-4,-0.166667},{12,-0.166667},{12,-46.1667},
              {-36.2,-46.1667}}, color={0,0,127}));
      connect(Ctrl1.contr_vars_real[4], heat_transfer_station1.contr_vars_real[
        4]) annotation (Line(points={{-4,0.166667},{12,0.166667},{12,-45.8333},
              {-36.2,-45.8333}}, color={0,0,127}));
      connect(Ctrl1.contr_vars_real[5], heat_transfer_station1.contr_vars_real[
        5]) annotation (Line(points={{-4,0.5},{12,0.5},{12,-45.5},{-36.2,-45.5}},
            color={0,0,127}));
      connect(Ctrl1.contr_vars_real[6], heat_transfer_station1.contr_vars_real[
        6]) annotation (Line(points={{-4,0.833333},{10,0.833333},{10,-20},{-44,
              -20},{-44,-45.1667},{-36.2,-45.1667}}, color={0,0,127}));
      connect(Ctrl1.states[1], heat_transfer_station1.states[1]) annotation (
          Line(points={{-28,-0.875},{-36,-0.875},{-36,-20},{14,-20},{14,-46.875},
              {4,-46.875}}, color={0,0,127}));
      connect(Ctrl1.states[2], heat_transfer_station1.states[2]) annotation (
          Line(points={{-28,-0.625},{-36,-0.625},{-36,-20},{14,-20},{14,-46.625},
              {4,-46.625}}, color={0,0,127}));
      connect(Ctrl1.states[3], heat_transfer_station1.states[3]) annotation (
          Line(points={{-28,-0.375},{-36,-0.375},{-36,-20},{14,-20},{14,-46.375},
              {4,-46.375}}, color={0,0,127}));
      connect(Ctrl1.states[4], heat_transfer_station1.states[4]) annotation (
          Line(points={{-28,-0.125},{-36,-0.125},{-36,-20},{14,-20},{14,-46.125},
              {4,-46.125}}, color={0,0,127}));
      connect(Ctrl1.states[5], heat_transfer_station1.states[5]) annotation (
          Line(points={{-28,0.125},{-36,0.125},{-36,-20},{14,-20},{14,-45.875},
              {4,-45.875}}, color={0,0,127}));
      connect(Ctrl1.states[6], heat_transfer_station1.states[6]) annotation (
          Line(points={{-28,0.375},{-36,0.375},{-36,-20},{14,-20},{14,-45.625},
              {4,-45.625}}, color={0,0,127}));
      connect(Ctrl1.states[7], heat_transfer_station1.states[7]) annotation (
          Line(points={{-28,0.625},{-36,0.625},{-36,-20},{14,-20},{14,-45.375},
              {4,-45.375}}, color={0,0,127}));
      connect(Ctrl1.states[8], heat_transfer_station1.states[8]) annotation (
          Line(points={{-28,0.875},{-36,0.875},{-36,-20},{8,-20},{8,-45.125},{4,
              -45.125}}, color={0,0,127}));
      connect(heat_transfer_station1.cold_prim, volume.ports[2]) annotation (
          Line(points={{-2,-64},{-2,-162},{41,-162},{41,-178}}, color={0,127,
              255}));
      connect(heat_transfer_station1.hot_prim, valve_for_test.port_b)
        annotation (Line(points={{-30,-64.2},{-30,-70},{64,-70},{64,-148}},
            color={0,127,255}));
      annotation (Diagram(graphics={                               Rectangle(extent={{-62,92},
                  {28,-82}},      lineColor={28,108,200})}));
    end Test_heat_transfer_station_production;

    model Test_heat_transfer_station_Consumption
      Modelica.Blocks.Sources.RealExpression realExpression1(y=273.15) annotation (
          Placement(transformation(
            extent={{-5,-6},{5,6}},
            rotation=270,
            origin={24,53})));
      Modelica.Blocks.Math.Add add1 annotation (Placement(transformation(
            extent={{-5,-5},{5,5}},
            rotation=-90,
            origin={1,39})));
      ProsNet.Under_Development.Controller_PID_based.auxiliary.TimeTable_noInterp
                                                        power_set2(table=[0,-10;
            3600,10; 7200,-4; 10800,4; 14400,10; 18000,-10; 21600,-4; 25200,-4])
                                                                   annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={-26,76})));
      ProsNet.Under_Development.Controller_PID_based.auxiliary.TimeTable_noInterp
                                                        temp_sec_in2(table=[0,
            30; 3600,55; 7200,30; 10800,55; 14400,55; 18000,30; 21600,30; 25200,
            30])                                                     annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={16,76})));
      ProsNet.Under_Development.Controller_PID_based.PID_Q_T_weighted_crossover
                                                      Ctrl2(
        alpha_prim_prod=1,
        alpha_sec_prod=0,
        alpha_prim_cons=0,
        alpha_sec_cons=1) annotation (Placement(transformation(extent={{-18,-14},
                {6,20}})));
      Modelica.Blocks.Sources.Ramp ramp(
        height=1,
        duration=600,
        offset=0,
        startTime=200)
        annotation (Placement(transformation(extent={{-90,-178},{-70,-158}})));
      Modelica.Fluid.Vessels.ClosedVolume volume(
        T_start=338.15,
        use_portsData=false,
        V=1,
        nPorts=2,
        redeclare final package Medium = ProsNet.Media.Water)
        annotation (Placement(transformation(extent={{4,-176},{24,-196}})));
      ProsNet.Fluid.Sources.Boundary_pT
                                bou(redeclare final package Medium =
            ProsNet.Media.Water, nPorts=1)
        annotation (Placement(transformation(extent={{108,-142},{88,-122}})));
      ProsNet.Fluid.Pumps.SpeedControlled_y
                                    pump_prim_prod(
        redeclare final package Medium = ProsNet.Media.Water,
        final energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
        final tau=1,
        redeclare final ProsNet.Fluid.Pumps.Data.Pumps.IMP.NMTPlus152025to40
          per,
        final use_inputFilter=true,
        final riseTime=5,
        final init=Modelica.Blocks.Types.Init.InitialOutput,
        final y_start=0)                annotation (Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=-90,
            origin={42,-144})));
      heat_transfer_station heat_transfer_station1
        annotation (Placement(transformation(extent={{-26,-62},{14,-26}})));
    equation
      connect(realExpression1.y,add1. u1) annotation (Line(points={{24,47.5},{
              24,45},{4,45}},             color={0,0,127}));
      connect(temp_sec_in2.y,add1. u2)
        annotation (Line(points={{16,65},{16,64},{-2,64},{-2,45}},
                                                              color={0,0,127}));
      connect(power_set2.y,Ctrl2. Q_dot_set) annotation (Line(points={{-26,65},
              {-26,30},{-12,30},{-12,20.8}},
                                   color={0,0,127}));
      connect(add1.y,Ctrl2. T_sec_in_is)
        annotation (Line(points={{1,33.5},{1,28},{0,28},{0,21}},
                                                               color={0,0,127}));
      connect(volume.ports[1],pump_prim_prod. port_a) annotation (Line(points={{13,-176},
              {13,-160},{42,-160},{42,-154}},      color={0,127,255}));
      connect(ramp.y,pump_prim_prod. y) annotation (Line(points={{-69,-168},{20,
              -168},{20,-144},{30,-144}},
                              color={0,0,127}));
      connect(pump_prim_prod.port_b, bou.ports[1]) annotation (Line(points={{42,
              -134},{42,-132},{88,-132}}, color={0,127,255}));
      connect(heat_transfer_station1.states[1], Ctrl2.states[1]) annotation (
          Line(points={{14,-44.875},{22,-44.875},{22,-20},{-26,-20},{-26,1.125},
              {-18,1.125}}, color={0,0,127}));
      connect(heat_transfer_station1.states[2], Ctrl2.states[2]) annotation (
          Line(points={{14,-44.625},{22,-44.625},{22,-20},{-26,-20},{-26,1.375},
              {-18,1.375}}, color={0,0,127}));
      connect(heat_transfer_station1.states[3], Ctrl2.states[3]) annotation (
          Line(points={{14,-44.375},{22,-44.375},{22,-20},{-26,-20},{-26,1.625},
              {-18,1.625}}, color={0,0,127}));
      connect(heat_transfer_station1.states[4], Ctrl2.states[4]) annotation (
          Line(points={{14,-44.125},{22,-44.125},{22,-20},{-26,-20},{-26,1.875},
              {-18,1.875}}, color={0,0,127}));
      connect(heat_transfer_station1.states[5], Ctrl2.states[5]) annotation (
          Line(points={{14,-43.875},{22,-43.875},{22,-20},{-26,-20},{-26,2.125},
              {-18,2.125}}, color={0,0,127}));
      connect(heat_transfer_station1.states[6], Ctrl2.states[6]) annotation (
          Line(points={{14,-43.625},{22,-43.625},{22,-20},{-26,-20},{-26,2.375},
              {-18,2.375}}, color={0,0,127}));
      connect(heat_transfer_station1.states[7], Ctrl2.states[7]) annotation (
          Line(points={{14,-43.375},{22,-43.375},{22,-20},{-26,-20},{-26,2.625},
              {-18,2.625}}, color={0,0,127}));
      connect(heat_transfer_station1.states[8], Ctrl2.states[8]) annotation (
          Line(points={{14,-43.125},{22,-43.125},{22,-20},{-26,-20},{-26,2.875},
              {-18,2.875}}, color={0,0,127}));
      connect(heat_transfer_station1.contr_vars_real[1], Ctrl2.contr_vars_real[
        1]) annotation (Line(points={{-26.2,-44.8333},{-34,-44.8333},{-34,-16},
              {-22,-16},{-22,-18},{14,-18},{14,1.16667},{6,1.16667}}, color={0,
              0,127}));
      connect(heat_transfer_station1.contr_vars_real[2], Ctrl2.contr_vars_real[
        2]) annotation (Line(points={{-26.2,-44.5},{-34,-44.5},{-34,-16},{-22,
              -16},{-22,-18},{14,-18},{14,1.5},{6,1.5}}, color={0,0,127}));
      connect(heat_transfer_station1.contr_vars_real[3], Ctrl2.contr_vars_real[
        3]) annotation (Line(points={{-26.2,-44.1667},{-34,-44.1667},{-34,-16},
              {-22,-16},{-22,-18},{14,-18},{14,1.83333},{6,1.83333}}, color={0,
              0,127}));
      connect(heat_transfer_station1.contr_vars_real[4], Ctrl2.contr_vars_real[
        4]) annotation (Line(points={{-26.2,-43.8333},{-34,-43.8333},{-34,-16},
              {-22,-16},{-22,-18},{14,-18},{14,2.16667},{6,2.16667}}, color={0,
              0,127}));
      connect(heat_transfer_station1.contr_vars_real[5], Ctrl2.contr_vars_real[
        5]) annotation (Line(points={{-26.2,-43.5},{-34,-43.5},{-34,-16},{-22,
              -16},{-22,-18},{14,-18},{14,2.5},{6,2.5}}, color={0,0,127}));
      connect(heat_transfer_station1.contr_vars_real[6], Ctrl2.contr_vars_real[
        6]) annotation (Line(points={{-26.2,-43.1667},{-34,-43.1667},{-34,-16},
              {-22,-16},{-22,-18},{14,-18},{14,2.83333},{6,2.83333}}, color={0,
              0,127}));
      connect(heat_transfer_station1.hot_prim, pump_prim_prod.port_b)
        annotation (Line(points={{-20,-62.2},{-20,-128},{42,-128},{42,-134}},
            color={0,127,255}));
      connect(heat_transfer_station1.cold_prim, volume.ports[2]) annotation (
          Line(points={{8,-62},{8,-166},{15,-166},{15,-176}}, color={0,127,255}));
      annotation (Diagram(graphics={                               Rectangle(extent={{-52,92},
                  {38,-82}},      lineColor={28,108,200})}));
    end Test_heat_transfer_station_Consumption;
  end Substation_idealProsumer;

  package Substation
  end Substation;
end BidirectionalSubstation;